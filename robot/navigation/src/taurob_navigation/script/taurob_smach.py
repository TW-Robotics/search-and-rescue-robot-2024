#!/usr/bin/env python
"""Creates the State Machine for the robot"""
import threading
import rospy
import smach
import smach_ros


from geometry_msgs.msg import PoseStamped

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult


class WaitForGoal(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["received_goal", "preempted"],
            input_keys=[],
            output_keys=["target_pose"],
        )

        self.target_pose = PoseStamped()
        self.signal = threading.Event()
        self.subscriber = None

    def execute(self, userdata):

        rospy.loginfo("Waiting for a goal...")
        self.signal.clear()
        self.subscriber = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )
        while (
            not rospy.is_shutdown()
            and not self.signal.is_set()
            and not self.preempt_requested()
        ):
            rospy.logdebug("Waiting for a goal...")
            self.signal.wait(1)

        if self.preempt_requested() or rospy.is_shutdown():
            self.service_preempt()
            return "preempted"

        userdata.target_pose = self.target_pose
        pos = self.target_pose.pose.position
        rospy.loginfo("Received goal pose: (%s, %s, %s)", pos.x, pos.y, pos.z)

        return "received_goal"

    def goal_callback(self, msg):
        rospy.logdebug("Received goal pose: %s", str(msg))
        self.target_pose = msg
        self.signal.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()

    def unsub(self):
        self.subscriber.unregister()


class Planning(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "preempted", "failure", "invalid", "aborted"],
            input_keys=["target_pose"],
            output_keys=["outcome", "message", "path"],
        )

        with self:
            state = smach_ros.SimpleActionState(
                "move_base_flex/get_path",
                GetPathAction,
                goal_cb=Planning.get_path_goal_cb,
                result_cb=Planning.get_path_result_cb,
            )
            smach.StateMachine.add(
                "GET_PATH",
                state,
                transitions={
                    "succeeded": "succeeded",
                    "failure": "failure",
                    "invalid": "invalid",
                    "preempted": "preempted",
                    "aborted": "aborted",
                },
            )  # ka warum ich aborted brauch

    @staticmethod
    @smach.cb_interface(input_keys=["target_pose"])
    def get_path_goal_cb(userdata, goal):
        # Use start_pose or current position as the beginning of the path
        goal.use_start_pose = False
        # The pose to achieve with the path
        goal.target_pose = userdata.target_pose
        goal.tolerance = 0.2  # 20cm tolerance to the target
        # Planner to use; defaults to the first one specified on "planners" parameter
        goal.planner = "GlobalPlanner"

    @staticmethod
    @smach.cb_interface(
        output_keys=["message", "outcome", "path"],
        outcomes=["succeeded", "failure", "preempted", "invalid"],
    )
    def get_path_result_cb(userdata, status, result):

        userdata.message = result.message
        userdata.outcome = result.outcome
        userdata.path = result.path

        if result.outcome == GetPathResult.SUCCESS:
            return "succeeded"
        elif result.outcome == GetPathResult.CANCELED:
            return "preempted"
        elif result.outcome in (
            GetPathResult.INVALID_START,
            GetPathResult.INVALID_GOAL,
            GetPathResult.NO_PATH_FOUND,
            GetPathResult.PAT_EXCEEDED,
        ):
            # We will take PAT_EXCEEDED as a failure to get a path. The most planners currently won't give a better feedback, so if it
            # takes too long to find a path, there might be no path.
            print(
                "Planning could not get any path %s:\n%s"
                % (str(result.outcome), result.message)
            )
            return "invalid"
        else:
            print(
                "Planning terminated with non-success status code %s:\n%s"
                % (str(result.outcome), result.message)
            )
            return "failure"


class Controlling(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=[
                "succeeded",
                "preempted",
                "failure",
                "aborted",
                "change_controller",
            ],
            input_keys=["path"],
            output_keys=[
                "outcome",
                "message",
                "final_pose",
                "dist_to_goal",
                "angle_to_goal",
                "recov_cont",
                "switch_controller",
                "second_controller",
            ],
        )

        with self:
            # self.userdata.recovery_behavior = 'clear_costmap'
            self.userdata.local_planer = "teb"
            self.userdata.recov_cont = 0
            self.userdata.switch_controller = False
            self.userdata.second_controller = False
            state = smach_ros.SimpleActionState(
                "move_base_flex/exe_path",
                ExePathAction,
                goal_cb=Controlling.exe_path_goal_cb,
                result_cb=Controlling.exe_path_result_cb,
            )
            smach.StateMachine.add(
                "EXEC_PATH",
                state,
                transitions={
                    "succeeded": "succeeded",
                    "failure": "failure",
                    "aborted": "aborted",
                    "preempted": "preempted",
                    "change_controller": "change_controller",
                },
            )

    @staticmethod
    @smach.cb_interface(input_keys=["path", "local_planer"])
    def exe_path_goal_cb(userdata, goal):
        goal.path = userdata.path
        goal.controller = userdata.local_planer

    @staticmethod
    @smach.cb_interface(
        input_keys=["recov_cont", "switch_controller", "second_controller"],
        output_keys=[
            "path",
            "outcome",
            "message",
            "final_pose",
            "dist_to_goal",
            "angle_to_goal",
            "local_planer",
            "recov_cont",
            "switch_controller",
            "second_controller",
        ],
        outcomes=["succeeded", "aborted", "failure", "preempted", "change_controller"],
    )
    def exe_path_result_cb(userdata, status, result):
        outcome_map = {
            ExePathResult.COLLISION: "COLLISION",
            ExePathResult.CANCELED: "CANCELED",
            ExePathResult.BLOCKED_PATH: "BLOCKED_PATH",
            ExePathResult.FAILURE: "FAILURE",
            ExePathResult.INTERNAL_ERROR: "INTERNAL_ERROR",
            ExePathResult.INVALID_PATH: "INVALID_PATH",
            ExePathResult.MISSED_GOAL: "MISSED_GOAL",
            ExePathResult.INVALID_PLUGIN: "INVALID_PLUGIN",
            ExePathResult.MISSED_PATH: "MISSED_PATH",
            ExePathResult.NO_VALID_CMD: "NO_VALID_CMD",
            ExePathResult.NOT_INITIALIZED: "NOT_INITIALIZED",
            ExePathResult.OSCILLATION: "OSCILLATION",
            ExePathResult.PAT_EXCEEDED: "PAT_EXCEEDED",
            ExePathResult.ROBOT_STUCK: "ROBOT_SUCK",
            ExePathResult.TF_ERROR: "TF_ERROR",
            ExePathResult.SUCCESS: "SUCCESS",
        }

        controller_aborted_map = [
            ExePathResult.TF_ERROR,
            ExePathResult.INTERNAL_ERROR,
            ExePathResult.INVALID_PATH,
            ExePathResult.NOT_INITIALIZED,
        ]

        controller_failed_map = [
            ExePathResult.PAT_EXCEEDED,
            ExePathResult.BLOCKED_PATH,
            ExePathResult.FAILURE,
            ExePathResult.MISSED_PATH,
            ExePathResult.MISSED_GOAL,
            ExePathResult.NO_VALID_CMD,
            ExePathResult.OSCILLATION,
            ExePathResult.ROBOT_STUCK,
        ]

        userdata.outcome = result.outcome
        userdata.message = result.message
        userdata.final_pose = result.final_pose
        userdata.dist_to_goal = result.dist_to_goal
        userdata.angle_to_goal = result.angle_to_goal

        # recovery_behavior = 'clear_costmap'
        if result.outcome == ExePathResult.SUCCESS:
            p = result.final_pose.pose.position
            rospy.loginfo(
                "Controller arrived at goal: (%s), %s, %s",
                str(p),
                outcome_map[result.outcome],
                result.message,
            )
            userdata.local_planer = "teb"
            userdata.recov_cont = 0
            # userdata.switch_controller = False
            userdata.second_controller = False
            return "succeeded"
        elif result.outcome == ExePathResult.CANCELED:
            rospy.loginfo("Controller has been canceled.")
            return "preempted"
        elif (
            result.outcome in controller_failed_map
            and userdata.switch_controller is True
        ):
            rospy.logwarn("Goal not reachable -> Switching Controller")
            userdata.switch_controller = False
            userdata.local_planer = "dwa"
            userdata.recov_cont = 0
            userdata.second_controller = True
            return "change_controller"
        elif result.outcome in controller_failed_map and userdata.recov_cont < 3:
            rospy.logwarn(
                "Controller failed: %s, %s", outcome_map[result.outcome], result.message
            )
            userdata.recov_cont = userdata.recov_cont + 1
            if userdata.recov_cont == 3 and userdata.second_controller is False:
                userdata.switch_controller = True
            return "failure"
        else:
            rospy.logfatal(
                "Controller aborted: %s, %s",
                outcome_map[result.outcome],
                result.message,
            )
            userdata.local_planer = "teb"
            userdata.recov_cont = 0
            # userdata.switch_controller = False
            userdata.second_controller = False
            return "aborted"


class Recovery(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "preempted", "aborted"],
            # input_keys=['recovery_behavior'],
            # output_keys=['outcome', 'message', 'recovery_behavior']
            output_keys=["recovery_behavior"],
        )

        with self:
            self.userdata.recovery_behavior = "clear_costmap"
            smach.StateMachine.add(
                "RECOVERY",
                smach_ros.SimpleActionState(
                    "move_base_flex/recovery",
                    RecoveryAction,
                    goal_cb=Recovery.recovery_goal_cb,
                    result_cb=Recovery.recovery_result_cb,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "preempted",
                    "aborted": "aborted",
                },
            )

    @staticmethod
    @smach.cb_interface(
        input_keys=["recovery_behavior"], output_keys=["recovery_behavior"]
    )
    def recovery_goal_cb(userdata, goal):
        rospy.loginfo("Recovery behavior: %s", userdata.recovery_behavior)
        goal.behavior = userdata.recovery_behavior
        rospy.loginfo("Recovery behavior set to: %s", goal.behavior)

        if userdata.recovery_behavior == "clear_costmap":
            userdata.recovery_behavior = "moveback_recovery"
        elif userdata.recovery_behavior == "moveback_recovery":
            userdata.recovery_behavior = "rotate_recovery"
        elif userdata.recovery_behavior == "rotate_recovery":
            userdata.recovery_behavior = "clear_costmap"

    @staticmethod
    @smach.cb_interface(
        output_keys=["outcome", "message", "recovery_behavior"],
        outcomes=["succeeded", "aborted", "preempted"],
    )
    def recovery_result_cb(userdata, status, result):

        if result.outcome == RecoveryResult.SUCCESS:
            rospy.loginfo("RECOVERY OUTCOME SUCCESS")
            return "succeeded"
        elif result.outcome == RecoveryResult.CANCELED:
            rospy.loginfo("RECOVERY OUTCOME CANCELED")
            return "preempted"
        else:
            rospy.loginfo("RECOVERY OUTCOME ABORTED SWITCHING MODE")
            # userdata.recovery_behavior = 'rotate_recovery'
            return "aborted"


def main():
    rospy.init_node("mbf_state_machine")

    SM = smach.StateMachine(outcomes=["aborted", "preempted"])
    # waitforgoal = None

    with SM:
        waitforgoal = WaitForGoal()
        smach.StateMachine.add(
            "WAIT_FOR_GOAL",
            waitforgoal,
            transitions={"received_goal": "PLANNING", "preempted": "preempted"},
        )
        smach.StateMachine.add(
            "PLANNING",
            Planning(),
            transitions={
                "succeeded": "CONTROLLING",
                "preempted": "preempted",
                "failure": "WAIT_FOR_GOAL",  # recovery
                "invalid": "WAIT_FOR_GOAL",
                "aborted": "WAIT_FOR_GOAL",  # ka warum ich des brauch
            },
        )
        smach.StateMachine.add(
            "CONTROLLING",
            Controlling(),
            transitions={
                "succeeded": "WAIT_FOR_GOAL",
                "preempted": "WAIT_FOR_GOAL",
                "failure": "RECOVERY",  # recovery
                "aborted": "WAIT_FOR_GOAL",
                "change_controller": "CONTROLLING",
            },
        )
        smach.StateMachine.add(
            "RECOVERY",
            Recovery(),
            transitions={
                "succeeded": "CONTROLLING",
                "preempted": "preempted",
                "aborted": "RECOVERY",
            },
        )

    sis = smach_ros.IntrospectionServer("mbf_state_machine_server", SM, "/MBF_SM")
    sis.start()

    SM.execute()

    waitforgoal.unsub()
    sis.stop()


if __name__ == "__main__":
    main()

# Systemd Files for Docker-Compose

Tutorial to start docker compose files on boot using systemd



Install
=======

1. Make sure to adapt `WorkingDirectory` to the path of the [robot](../../robot) folder (eg. `/home/mluser/git/taurob_tracker/robot/`).
2. Symlink in `/etc/systemd/system/` the [taurob@.service](./taurob@.service) file  (the `@` is used as a substitute character so that we can start multiple daemons using one .service file)
3. Enable the service
4. Create a symlink of [check_ros.sh](check_ros.sh) into the folders where Dockerfiles are located.


```shell
cd /etc/systemd/system && sudo ln -s /home/mluser/git/taurob_tracker/robot/systemd/taurob@.service
sudo systemctl enable taurob@navigation && sudo systemctl start taurob@navigation.service
systemctl status taurob@navigation
ln -s check_ros.sh ../navigation/check_ros.sh
```


JournalD support
================

Just add the following line to the `/etc/docker/daemon.json`:

```json
{
    ...
    "log-driver": "journald",
    ...
}
```

Get logs: `sudo journalctl -w  CONTAINER_NAME=sensing_velodyne_1`
or: `journalctl -u taurob@navigation.service -g`

Check ROS 
=========
Systemd ExecStartPre skript that monitors the roscore and restarts the service if roscore is stopped.
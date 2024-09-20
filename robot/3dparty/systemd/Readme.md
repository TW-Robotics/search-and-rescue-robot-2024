# Systemd Files for Docker-Compose

Tutorial to start docker compose files on boot using systemd



Install
=======

1. Make sure to adapt `WorkingDirectory` to the path of the [3dparty](../../3dparty/) folder (eg. `/home/mluser/git/taurob_tracker/robot/3dparty/`).
2. Symlink in `/etc/systemd/system/` the [compose@.service](./compose@.service) file  (the `@` is used as a substitute character so that we can start multiple daemons using one .service file)
3. Enable the service
4. Create a symlink of [check_ros.sh](check_ros.sh) into the folders where Dockerfiles are located.


```shell
cd /etc/systemd/system && sudo ln -s /home/mluser/git/taurob_tracker/robot/3dparty/systemd/compose@.service
sudo systemctl enable compose@sensing && sudo systemctl start compose@sensing.service
systemctl status compose@sensing
ln -s check_ros.sh ./sensing/check_ros.sh
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
or: `journalctl -u compose@sensing.service -g`

Check ROS 
=========
Systemd ExecStartPre skript that monitors the roscore and restarts the service if roscore is stopped.
Software autorun
===

> **Note** In the image version **0.20** `clever` package and service was renamed to `technic`. See [previous version of the article](https://github.com/CopterExpress/technic/blob/v0.19/docs/en/autolaunch.md) for older images.

systemd
---

Main documentation: https://wiki.archlinux.org/title/Systemd.

All automatically started Technic software is launched as a `technic.service` systemd service.

The service may be restarted by the `systemctl` command:

```(bash)
sudo systemctl restart technic
```

Text output of the software can be viewed using the `journalctl` command:

```(bash)
journalctl -u technic
```

To run Technic software directly in the current console session, you can use the `roslaunch` command:

```(bash)
sudo systemctl restart technic
roslaunch technic technic.launch
```

You can disable Technic software autolaunch using the `disable` command:

```(bash)
sudo systemctl disable technic
```

roslaunch
---

Main documentation: http://wiki.ros.org/roslaunch.

The list of nodes / programs declared for running is specified in file `/home/pi/catkin_ws/src/technic/technic/launch/technic.launch`.

You can add your own node to the list of automatically launched ones. To do this, place your executable file (e.g. `my_program.py`) into folder `/home/pi/catkin_ws/src/technic/technic`. Then add the start of your node to `technic.launch`, for example:

```xml
<node name="my_program" pkg="technic" type="my_program.py" output="screen"/>
```

The started file must have *permission* to run:

```bash
chmod +x my_program.py
```

When scripting languages are used, a <a href="https://en.wikipedia.org/wiki/Shebang_(Unix)">shebang</a> should be placed at the beginning of the file, for example:

```bash
#!/usr/bin/env python3
```

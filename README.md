# Robotik-Projekt-2-Team-Follow
Setup Bedingungen:

Brettübergänge müssen fließend sein, Kanten führen zu schlagartigen Richtungsänderungen

Als Beleuchtung sollt kaltes Licht verwendet werden, dessen Intensität möglichst so eingestellt wird, dass die Oberflächen nicht reflektieren.

Roboter sollte nicht auf einem RFID-Tag gestartet werden, da er sonst bereits ein Wendemanöver durchführen wird.

Startup:

Um den Roboter zu starten werden 2 Terminals benötigt.

In beiden muss zuerst der Befehl: 'ssh ubuntu@10.42.0.1' ausgeführt werden und anschließend das Passwort für den TurtleBot eingegeben werden.

Im ersten Terminal muss der Befehl: 'ros2 launch turtlebot3_bringup robot.launch.py' ausgeführt werden.

Im zweiten Terminal:
1. 'cd ros2_ws_bot'
2. 'source install/setup.bash'
3. 'cd src/launch'
4. 'ros2 launch ro36_move_control_follow.py

Abbruch:

In beiden Terminals strg + c


# COME VISUALIZZARE TF
--> runnare la vnc prima di runnare ROS
--> eseguire roscore in una tmux session
--> in un'altra sessione (o nella stessa) runnare la bag: cd src/bags
					 		  rosbag play --clock --pause robotics.bag
--> in un altro pannello lanciare: roslaunch first_package launch.launch
--> tornare al pannello della bag e clickare spazio per farla partire

//DA QUI IN POI SUPERFLUO, DOVREBBE ESSERE GIà TUTTO IMPOSTATO GRAZIE AL CONFIG FILE
--> aprire la vnc (localhost:8080), dovrebbe già essersi aperto rviz e dovrebbe essere già settato in alto a sinistra il fixed frame a "world".
--> Clickare in basso a destra su add, scrollare fino in fondo e clickare su tf
--> nel pannello di destra in alto selezionare Type: TopDownOrtho (rviz)

## COSA SI DOVREBBE VEDERE:
Dovrebbero visualizzarsi gli assi del sistema di riferimento "world" fissi in centro alla griglia e altri due sistemi di riferimento
("wheel_odom" e "gps_odom") che si muovono nel piano e divergono dopo un po' di tempo. 
In particolare ho notato che nel periodo 120s-230s della bag sono molto diversi e gps_odom sembra smettere di seguire il wheel_odom.
Dopo i 230s tornano a muoversi in modo simile anche se in posizioni abbastanza diverse, dopodiché dai 350s in poi divergono un'altra volta.

# TODO
- [Terzo nodo]
- [Eventualmente sistemare primo nodo]
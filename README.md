# Nota sulla visualizzazione
Eseguire tutti i nodi e i tool grafici è sufficiente scrivere il comando:
`roslaunch first_project launch.launch`
Tuttavia, nel caso in cui non sia ancora stato eseguito il comando che avvia la riproduzione della bag, il tf_tree risulterà vuoto.

Il comando per eseguire la bag è il seguente:
`rosbag play --clock robotics.bag`

Dopo che viene eseguito questo comando è possibile visualizzare in rviz i movimenti dei diversi reference frames uno rispetto all'altro.
È inoltre possibile riconfigurare dinamicamente il reference frame del pointcloud da wheel_odom a gps_odom.
Infine, per visualizzare il tf_tree potrebbe essere necessario "refreshare" l'interfaccia tramite il simbolo delle frecce in alto a sinistra.

## Divergenza tra gps odom e wheel odom
Dovrebbero visualizzarsi gli assi del sistema di riferimento "world" fissi in centro alla griglia e altri due sistemi di riferimento
("wheel_odom" e "gps_odom") che si muovono nel piano e divergono dopo un po' di tempo.
In particolare ho notato che nel periodo 120s-230s della bag sono molto diversi e gps_odom sembra smettere di seguire il wheel_odom.
Dopo i 230s tornano a muoversi in modo simile anche se in posizioni abbastanza diverse, dopodiché dai 350s in poi divergono un'altra volta.

# youbot-xsens-controller

## Descripción

**youbot-xsens-controller** es un stack de ROS (*Robot Operating System*) que incluye las herramientas necesarias para la toma de datos de IMUs xsens y el control del brazo robótico del robot Youbot de KUKA. También es posible controlar el simulador del robot para el programa Gazebo gracias a que tanto el robot como el simulador escuchan los mismos topics de ROS.

## Componentes del stack

Este stack está dividido en tres paquetes:

* **xsens_driver**: Paquete para la toma de datos de un IMU xsens o una red de IMUs xsens conectados al master xbus y su publicación en ROS. El programa `xsens_node` detecta el número de sensores conectados al PC y los configura para que devuelva los siguientes datos:
    
    * Vector aceleración lineal.
    * Vector velocidad angular.
    * Vector campo magnético.
    * Cuaternión de orientación.
    
 Se puede modificar este programa para obtener otro tipo de datos. Más información en al apartado dedicado a este paquete.

* **youbot_controller**: Paquete para el control de cada una de las articulaciones del brazo robótico y la posición del gripper a través de topics de ROS. El programa `youbot_controller` está preparado para subscribirse a los topics que contienen los cuaterniones de orientación de tres sensores xsens publicados por el programa `xsens_node`. Más información en el apartado dedicado a este paquete.

* **dfv**: Librería para trabajar con cuaterniones de orientación, vectores y matrices. El programa `example` contiene ejemplos de cómo usar la librería. Más información en el apartado dedicado a este paquete.

## Instalación

Este stack se ha realizado en Ubuntu 12.04 LTS y ROS Fuerte. Es posible que no funcione en otras versiones de Ubuntu o ROS. 

Para instalar el paquete simplemente nos situaremos en nuestro espacio de trabajo de ROS y clonaremos el repositorio:

    $ git clone https://github.com/DaniSagan/youbot-xsens-controller

### Instalación de las dependencias

Para la toma de datos del sensor xsens con el paquete `xsens_driver` no nos hará falta instalar ninguna cosa más. Sin embargo el paquete `youbot_controller` hace uso del paquete `brics_actuator` para el intercambio de mensajes con el robot o el simulador. El robot Youbot ya incorpora este paquete en su instalación de fábrica, pero si queremos usar el simulador en un PC normal lo más seguro es que no lo tengamos instalado. Si no tuviéramos dicho paquete, realizaremos su instalación de la siguiente forma:

    $ sudo apt-get install ros-fuerte-pr2-controllers
    $ sudo apt-get install ros-fuerte-pr2-simulator
    
Nos situamos en nuestro espacio de trabajo:
    
    $ git clone https://github.com/youbot/youbot-ros-pkg.git
    $ git clone https://github.com/ipa320/cob_common.git
    $ rospack profile
    $ rosmake brics_actuator youbot_description
    
### Compilación

Compilamos el stack:

    $ rosmake youbot-xsens-controller

## Puesta en funcionamiento

Una vez instalado y compilado el stack `youbot-xsens-controller` con sus dependencias, podemos lanzar los programas de ejemplo de cada paquete para ver su funcionamiento.

### Control del simulador del robot Youbot en un PC normal 

Conectamos el xbus master con tres sensores xsens al puerto USB del ordenador y lanzamos ROS:

    $ roscore
    
Ejecutamos el programa `xsens_node`:

    $ rosrun xsens_driver xsens_node
    
Lanzamos el simulador del brazo del robot:

    $ roslaunch youbot_description youbot_arm_publisher.launch
    
Si lo estimáramos conveniente, colocaremos el brazo en una posición más cómoda. Ahora ejecutamos el controlador:

    $ rosrun youbot_controller youbot_controller
    
Si todo ha ido bien, podremos controlar el simulador del brazo moviendo los sensores.    
    
### Control del robot real desde el PC embebido del Youbot

Conectamos el xbus master con tres sensores xsens al puerto USB del Youbot y en el PC embebido lanzamos ROS:

    $ roscore
    
Ejecutamos el programa `xsens_node`:

    $ rosrun xsens_driver xsens_node
    
Lanzamos el driver del robot una vez hayamos puesto en marcha el brazo robótico (Si necesitáramos permisos de usuario nos logeamos como superuser con el comando `$ su -`):

    $ roslaunch youbot_oodl youbot_oodl_driver.launch
    
Ahora ejecutamos el controlador:

    $ rosrun youbot_controller youbot_controller
    
Si todo ha ido bien, podremos controlar el brazo real moviendo los sensores. 

## Descripción de los programas de ejemplo

* **xsens_node**: Este programa realiza un escaneo de los puertos del PC y detecta cuántos sensores hay conectados. A continuación los configura para que devuelvan los datos calibrados y el cuaternión de orientación (configuración por defecto), y asigna una matriz de prerrotación unitaria. Para cada sensor publica los siguientes topics:
    
    ```bash
    /xsens_node/sensorX/acc : vector aceleración    
    /xsens_node/sensorX/gyr : vector velocidad angular    
    /xsens_node/sensorX/mag : vector campo magnético    
    /xsens_node/sensorX/ori_quat : cuaternión de orientación    
    ```
    
    Donde X es el número de identificación de cada sensor (0, 1, 2...).    
    Si se hubiera establecido otra configuración, los posibles topics son los siguientes:
    
    ```bash
    /xsens_node/sensorX/raw_acc : vector aceleración (raw)    
    /xsens_node/sensorX/raw_gyr : vector velocidad angular (raw)    
    /xsens_node/sensorX/raw_mag : vector campo magnñetico (raw)    
    /xsens_node/sensorX/pos_lla : posición (latitud, longitud, altitud)    
    /xsens_node/sensorX/ori_matrix : matriz de orientación    
    /xsens_node/sensorX/ori_euler : ángulos de Euler    
    ```
    
    También se publican los siguientes valores en el servidor de parámetros de ROS:

    ```bash
    /xsens_node/sensor_count : número de IMUs conectados    
    /xsens_node/sensorX/output_mode : modo de salida    
    /xsens_node/sensorX/output_settings : configuración de salida
    ```    

* **youbot_controller**: Este programa comprueba que el número de sensores coectados es 3 (parámetro `/xsens_node/sensor_count`) y obtiene sus cuaterniones de orientación (topic `/xsens_node/sensorX/ori_quat`). Realiza el cálculo de los ángulos que hay entre cada sensor (roll, pitch y yaw) y publica las posiciones de cada articulación en el topic `/arm_1/arm_controller/position_command` y la posición del gripper en el topic `arm_1/gripper_controller/position_command`

* **example**: En el código de este programa se pueden ver ejemplos de operaciones y funciones que hacen uso de las clases Vector3, Quaternion y Matrix de la librería dfv. Estas clases son usadas por el programa `youbot_controller` para la obtención de los ángulos entre sensores.

## Solución de problemas

### No se detectan sensores xsens a pesar de estar conectados

Al conectar los sensores es posible que haya que esperar 5 - 10 segundos antes de que el programa `xsens_node` sea capaz de detectarlos. Si tras dos o tres intentos aún no los detecta, puede que nuestro usuario de Ubuntu no pertenezca al grupo `dialout` y por lo tanto no tenga privilegios para leer el puerto serie. Esto podemos comprobarlo con el siguiente comando:
    
    $ groups
    
Si no vemos el grupo `dialout` en la salida de este comando, ejecutamos lo siguiente:


    $ sudo adduser <nombre_de_usuario> dialout
    
Sustituyendo `<nombre_de_usuario>` por nuestro nombre (p. ej. `$ sudo adduser daniel dialout`)Y reiniciamos el ordenador para que el cambio tenga efecto. Ahora el programa `xsens_node` tendría que detectar los sensores.

### El driver youbot_oodl no detecta el brazo del robot

Este error puede deberse a que necesitemos permisos de superusuario. Para logearnos como superusuario ejecutamos el siguiente comando:

    $ su -
    
### Al lanzar la simulación del robot no se inicia Gazebo

Es necesario añadir la siguiente línea al archivo `youbot-ros-pkg/youbot_common/youbot_description/launch/youbot_arm_publisher.launch`:
    
    <node name="gazebo_gui" pkg="gazebo" type="gui" respawn="false" output="screen"/>

Después de lo siguiente:

    <node name="gazebo" pkg="gazebo" type="gazebo" args="$(find gazebo_worlds)/worlds/empty.world" respawn="false" output="screen">
		    <env name="GAZEBO_RESOURCE_PATH" value="$(find youbot_description):$(find gazebo_worlds):$(find gazebo)/gazebo/share/gazebo" />
	</node>

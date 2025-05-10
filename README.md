# {Tutorial: Pick and place con ROS y Gazebo}
El siguiente tutorial tiene como objetivo desarrollar la simulación de un "Pick and Place" para el brazo robótico industrial de 6 grados de libertad con muñeca esférica "ABB IRB 140", todo el programa es desarrollado a través del sistema operativo robótico "ROS" y el simulador 3D de robótica "Gazebo", además de la herramienta de visualización en 3D para robots "RViz".
A lo largo de este tutorial aprenderás a cofigurar y simular el robot industrial **ABB IRB 140** en ROS y Gazebo. A lo largo del proceso:
- Crearás un workspace.
- Clonarás repositorios.
- Aprenderás a lanzar el robot en **RViz** y **Gazebo** correctamente.
- Ejecutarás movimientos tipo *pick and place*.
---

## 🧰 Requisitos Previos
Este tutorial esta pensado para personas que tiene poco o nula experiencia previa con ROS, pero que tienen interés en aprender cómo simular un robot industrial desde cero. Antes de comenzar, necesitas cumplir con lo siguiente:

💻 **Requisitos de hardware**
- Computadora con al menos 2 núcleos de procesador
- Contar 30 a 40 GB de espacio libre en disco duro
- 8 GB de RAM como mínimo
  
📋 **Requisitos de software**
- Sistema operativo: **Ubuntu 20.04** (versión compatible con ROS Noetic)
- Conexión a internet estable
- Editor de texto (VS Code, Gedit, o el que prefieras)
  
🔧 **Instalaciones necesarias**
**NOTA:** Si no tienes instalado nada de esto, puedes ver diferentes tutoriales como [Install Ubuntu Desktop | Ubuntu](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- ROS Noetic
- Gazebo
- RViz
✅ No necesitas saber programar en ROS o conocer el robot ABB IRB 140. Este tutorial te explicará desde lo más básico cómo configurarlo, moverlo y simular una tarea tipo pick and place.
---

## 📖  Introducción
**ABB Robotics** es una empresa líder en automatización industrial, reconocida por el desarrollo de soluciones robóticas avanzadas para entornos de manufactura exigentes. 
Uno de sus modelos es el **ABB IRB 140**, un robot industrial compacto de seis grados de libertad, diseñado para operar en espacios reducidos con alta velocidad, precisión y repetibilidad. Es ampliamente empleado en tareas como ensamblae, soldadura, manipulación de piezas y mucnas otras aplicaciones en espacios reducidos. 
![IRB140](https://github.com/EvaItzcoatl/ABB-IRBB-140-ROS-y-Gazebo-/blob/main/media/ABB_IRB_140_0001.png)

Sabiendo esto, el tema que abordaremos en este tutorial es la simulación del *ABB IRB 140** en un entorno virtual utilizando ROS y Gazebo, con el objetivo de diseñar y programar un rutina de **"Pick and Placle"**. Esta rutina es una de las tareas más comunes en robótica industrial, y consiste en tomar un objeto de un punto A y colocarlo en un punto B, de forma automática, precisa y repetitiva. 
Esto mediante la programción de los movimientos de los jonits, simulando movimientos como:
- MoveL: Movimiento en línea recta, se ocupa para movimientos, como subir y bajar, donde el robot se mueve de la forma más rápida y efectiva posible.
- MoveJ: Rotación de las juntas del robot para una trayectoría mása rápida.
Además, debemos tener en cuenta que al robot se le pueden agregar diferentes tipos de herramientas dependiendo de la actividad que se quiere realizar.
Sin embargo, para este tutorial, no se logró incorportar la herramienta (pinzas de agarre), por lo que se colocó un rectangulo al final del brazo para simualr el uso de la herramienta.

Este tutorial es ideal para quienes desean iniciarse en la robótica industrial desde la siulación, sin necesidad de contar con el robot físico. A lo largo de los pasos, irás comprendiendo cómo controlar los movimientos de forma programada.

---

## 🛠️ Configuración del Entorno

Pasos para configurar el entorno de desarrollo:

* Abriremos la terminal en Ubuntu

* Es necesario eliminar el catkin_ws que se tiene

* Crea un nuevo workspace (para este tutorial se nombro **prueba1_ws**
  
---

## 🏗️ Instrucciones

**NOTA:** Todos estos pason se deben realizar en la terminal de Ubuntu

**Paso 1:** Crear un nuevo workspace

Es recomendable eliminar el "catkin_ws" generado completamente y volver a generarlo desde cero.
Abrimos la terminal y colocaremos los siguientes comando, recuerda colocarlo uno por uno y presionar "enter" en cada comando. Como ejemplo se utilizara el nombre de prueba1_ws para el workspace, pero debes cambiarlo por el nombre de tu preferencia (debe terminar con _ws)
- mkdir -p ~/prueba1_ws/src 
- cd ~/prueba1_ws/src #Entrar a la carpeta src
- catkin_init_workspace #Crea un archivo simbolico CMakeLists.txt
- cd ~/prueba1_ws #Regresas a la carpeta del ws
- catkin_make #Genera los archivos build y devel, además de preparar el entorno de desarrollo
  
**Paso 2:** Eliminar archivos

Una vez terminado el "catkin_make" entramos a la carpeta del ws (workspace) y eliminamos las carpetas **"build"** y **"devel"**

**Paso 3:** Clonar repositorio

Para esto es necesario regresar a la carpeta personal
- cd #Te dirige a tu carpeta personal
- git clone https://github.com/EvaItzcoatl/ABB-IRB140-ROS-y-Gazebo.git #Copia el repositorio en tu carpeta personal

**Paso 4:** Mover archivos

Abrimos la carpeta que fue duplicada en tu carpeta personal con el nombre de **ABB-IRB140-ROS-y-Gazebo"**, dentros de esta carpeta vamos a seleccionar las sub-carpetas:
- build
- devel

Vamos a dar clic derecho y seleccionamos la opción **"mover a"**
Se debe abrir una nueva ventana, donde seleccionaremos:
- Carpeta personal
- prueba1_ws

Dentro de la carpeta **ABB-IRB140-ROS-y-Gazebo** tenemos la carpeta **"src"**, aquí podras encontrar un archivo con el nombre de **"abb_irb140_support"**:
- mover a
- Carpeta personal
- prueba1_ws
- src
- Le damos al botón verde que dice **"Seleccionar"**

Ahora ya podemos eliminar la carpeta completa de **"ABB-IRB140-ROS-y-Gazebo"**.
En la terminal, colocamos:
- cd ~/prueba1_ws
- rm -rf build devel #Elimina algunas configuraciones para evitar errores
- catkin_make #Prepara el entorno de desarrollo

**Paso 5:** Ejecución de Gazebo

En la terminal
- cd
- cd ~/prueba1_ws
- catkin_make
- source devel/setup.bash #Permite utilizar comandos ROS, asegura las conecciones entre los archivos y configura el entorno de trabajo
- roslaunch abb_irb140_support irb140_gazebo.launch #Este archivo abre Gazebo y muestra el robot
Con esto se abrira Gazebo y lograremos visualizar el robot ABB IRB140, no te preocupes si el robot aparece en con si estuviera tirado o mal acomodado, lo arreglaremos en los siguientes pasos.

**Paso 6:** Poner en posición cero el robot

Abre otro terminal y coloca los siguientes comandos:
- rosservice call /gazebo/pause_physics "{}" #Pausa la simulación física en Gazebo
- rosservice call /gazebo/dele_model "model_name: 'irb140'" #Elimina el robot
- rosparam get /robot_description #Permite simular el comportamiento del robot en Gazebo
- rosrun gazebo_ros spawn_model -urdf -param robot_description -model irb140 -x 0 -y 0 -z 0 #Coloca el robot en Gazebo y lo pone en configuración cero
- 
**Paso 7:** Mover el robot mediante un código en Phyton

Debemos entrar a la carpeta "scripts", para esto colocamos en la terminal lo siguiente:
- cd ~/prueba1_ws/src/abb_irb140_support/scripts/
- ./mover_irb140.py
Para poder ver el robor funcionar, es recomendable minimizar la terminal justa después de darle enter al comando.
Si sale algun error como **Permiso denegado** es necesario ir a la carpeta donde se encuentra el archivo (prueba1_ws/src/abb_irb140_support/scripts), le damos clic derecho al archivo y en seleccionamos *"propiedades"*, esto abre una nueva ventana, le damos en *"Permisos"* y seleccionamos **"Permitir ejecutar el archivo como programa"**.

**Explicacaión del código** 

  
---
## ✅ Conclusión
En este tutorial aprendimos las distntas formas de mover el robot ABB IRB 140 desde la terminal, mediante un código en Phyton o desde la terminal. De igual forma aprendimos un poco sobre el uso de ROS, como clonar un repositorio y crear carpetas o archivos desde la terminal, además de utilizar otras herramientas externas como lo es MATLAB. 
Este proyecto tiene como principal proposito enseñar a simular un "pick and place" usando el robot ABB IRB140 en ROS y Gazebo, mostrando comando para poder llamar archivos y lazar el robot, además de algunos comandos extras para poder modificar errores que podría presentar Gazebo. 
⚙️ Mejoras o proyectos a futuro
- Por cuestiones de tiempo ya no se inetento mover la simulación con MoveIt y RViz, por lo que se espera mejorar este proyecto para poder también mover el robot físico. 
- Implementar una alternativa de vinular MATLAB con ROS, pasa poder calcular los puntos en MATLAB y mandarlos a ROS para poder mover la simulación.
- Encontrar el semejante de jtraj de MATLAB para Phyton3.

---

## 📚 Referencias y Recursos Adicionales
Los siguientes repositorios fueron utilizados para generar el workspace utilizado. 
[Alexic12/ABB_IRB140_PACKAGES_ROS](https://github.com/Alexic12/ABB_IRB140_PACKAGES_ROS)
[FreddyMartinez/abb_irb_support](https://github.com/FreddyMartinez/abb_irb140_support)

---

## 📬 Contacto

Para preguntas o sugerencias:

* Redactor del tutorial: Eva Andrea Itzcoatl Tepeyahuitl
  
   📧 Correo electrónico: itzcoatl2902@gmail.com
  
---

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

**Este tutorial es ideal para quienes desean iniciarse en la robótica industrial desde la siulación, sin necesidad de contar con el robot físico. A lo largo de los pasos, irás comprendiendo cómo controlar los movimientos de forma programada.**
---

## 🛠️ Configuración del Entorno

Pasos para configurar el entorno de desarrollo:

* Abriremos la terminal en Ubuntu

* Es necesario eliminar el catkin_ws que se tiene

* Crea un nuevo workspace (para este tutorial se nombro **prueba1_ws**
  
---
## 🏗️ Instrucciones
**NOTA: Todos estos pason se deben realizar en la terminal de Ubuntu**
**Paso 1:** Crear un nuevo workspace
Colocar el siguiente comando en la terminal, deben colocarse uno por uno.
- mkdir -p ~/prueba1_ws/src
- cd ~/prueba1_ws/src
- catkin_init_workspace
- cd ~/prueba1_ws
- catkin_make
Al finalizar, en tus archivos podras ver una carpeta con el nombre de "prueba1_ws", dentro de esta carpeta encontraras las siguientes sub-carpetas:
- build
- devel
- src
**Paso 2:** Clonar repositorio en la carpeta src
- cd ~/prueba1_ws/src
- git clone https://github.com/EvaItzcoatl/ABB-IRBB-140-ROS-y-Gazebo-.git
Regresa a la raíz del workspace y compila
- cd ~/prueba1_ws
- catkin_make
**Paso 3:** Eliminación de archivos
Dentro de la carpeta "prueba1_ws":
- Elimina la carpeta "media"
- Abre la sub-carpeta src:
    --  Elimina las carpetas de build y devel
    -- Mueve los archivos que estan en la carpeta scr y colocalos dentro de la carpeta "prueba1_ws"

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

[Alexic12/ABB_IRB140_PACKAGES_ROS](https://github.com/Alexic12/ABB_IRB140_PACKAGES_ROS)
[FreddyMartinez/abb_irb_support](https://github.com/FreddyMartinez/abb_irb140_support)

---

## 📬 Contacto

Para preguntas o sugerencias:

* Redactor del tutorial: Eva Andrea Itzcoatl Tepeyahuitl
  
   📧 Correo electrónico: itzcoatl2902@gmail.com
  
---

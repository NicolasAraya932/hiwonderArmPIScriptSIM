El siguiente archivo presenta los codigos en Python para el funcionamiento de
la simulacion.

Requerimientos:
CoppeliaSimEdu
Python39 en adelante


Ya estan en la carpeta los archivos necesarios para conectarse como
remoteApi.dll, asyncronic.py,
sim.py, simConst.py, simpleSynchronousTest.py,
simpleTest.py.


Debe abrir en un IDE de preferencia el archivo de Python main.py
dentro de este se tiene separado por regiones cada parte importante del codigo.

I. Regiones de main.py

Modules: Las librerias usadas.
Empty: Arrays para guardar datos que posteriormente serian graficados.
CONNECT TO SERVER: Codigo para conectarse con CoppeliaSimEdu.
(MUY IMPORTANTE ABRIR COPPELIA ANTES Y TENER EL AMBIENTE LISTO)
SyncSim: Corresponde a unos comandos para que exista una sincronizacion con la
simulacion.
PARAMETERS: Todos los parametros necesarios para que funcione.
HANDLERS: Obtencion de Joints y otros Objetos dentro de la simulacion.
TRAJECTORY TO FOLLOW: Aqui se da la trayectoria que el brazo seguira.
Initial Velocity: Velocidad inicial de los Joints
TCONTROL: Control de seguimiento de trayectoria para llevar efector a
punto inicial.
Save: Se encuentra en varias partes del codigo y corresponde a
los comandos para guardar en los arrays creados en la region Empty.
SEGUIMIENTO DE TRAYECTORIA: Una vez termine TCONTROL seguira la trayectoria
pedida en la region TRAJECTORY TO FOLLOW. (Segun la norma de las distancias de los puntos
vuelve a usar TCONTROL para llegar de forma controlada al siguiente punto)
REFERENCIAS: Dentro de SEGUIMIENTO DE TRAYECTORIA y obtiene los valores necesarios
para que funcione la ley de control.
CONTROL LAW: Dentro de SEGUIMIENTO DE TRAYECTORIA, es la ley de control. (Se encuentra
en initializer.py)
SAVE TO EXCEL: Guarda en csv las posiciones y velocidades.
PLOTS: Grafica resultados.
STOP SIM: Detiene la Simulacion.

II. Trayectorias Cuadrada y Circular:

En la region TRAJECTORY TO FOLLOW tiene la opcion de agregar una trayectoria
hecha por imagenes dibujadas con lapiz rojo (De preferencia un plumon) o imagenes de contorno
negro, como dibujos de caricaturas.
Viene con 3 trayectorias por defecto la cuadrada y la circular. Basta con comentar las otras trayectorias
y descomentar la que va a utilizar.

Para mas informacion consultar Trayectorias.py

III. Trayectorias por imagenes:

En el espacio de trabajo existe la carpeta images. Dentro de esta carpeta se pueden poner las imagenes que se desee,
pero escogeria la que a su criterio sea la primera. Debido a ello, poner solo la imagen objetivo, osea, la imagen para
obtener la trayectoria.

Dependiendo de la imagen, si es dibujada con lapiz plumon rojo o tiene el contorno
como la imagen ejemplo de doraemon se debera cambiar la siguiente funcion:
ic.Drawed_Image_Processor('images') #Para dibujadas reciben como argumento la carpeta.
ic.Image_Processor('images') #Para contorno negro
Si no se respetan las funciones no creara trayectoria esperada.

Para mas informacion del procesamiento de las imagenes consultar image_control.py.



Se podra ver como se controlan los joints, objetos y la ley de control en initializer.py.

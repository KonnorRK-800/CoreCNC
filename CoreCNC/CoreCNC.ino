#include <StepperSR.h>

const int bufferSize = 64; // Tamaño del buffer de lectura del puerto serie
char buffer[bufferSize]; // Buffer de lectura del puerto serie

//Posicion actual del cabezal

float XYZ[3] = {0, 0, 0}; //Array donde se guardan las coordenadas xyz

//Informacion Miscelanea

int ElevaZ=10; //Indica cuanto debe elevarse en el eje z el cabezal, con respecto a la altura actual, antes de regresar al origen.

//Informacion del ratio de dezplazamieno de los ejes (paso/mm):

float StpPormm[3]={20,20,20};

//Informacion del motor Paso a paso:

int StpsPRev[3]={200,200,200}; //Aqui se indican la cantodad de pasos necesarios para que cada motor de una vuelta.

//Constructores de los motores:

StepperSR EjeX(StpsPRev[0], 0, 1, 2, 3);   //
StepperSR EjeY(StpsPRev[1], 4, 5, 6, 7);   // Constructores de los motores paso a paso
StepperSR EjeZ(StpsPRev[2], 8, 9, 10, 11); //

//Informacion recibida del puerto serie

float ValRec[10];  //Almacena los valores flotantes de los parametros recibidos
bool Presencia[10];//Dicatamina si dicho parametro estaba presente en el comando recibido

//El orden de los datos almacenados son: X,Y,Z,I,J,K,F,S,R,P

void procesamiento_de_entrada() {

  char* C; //Puntero que apunta a la direccion de memoria del caracter actualmente buscado
  char Caracteres[] = {'X', 'Y', 'Z', 'I', 'J', 'K', 'F', 'S', 'R','P'}; //Array que lista los caracteres buscados en el buffer

  for (int i = 0; i < 10; i++) {
    C = strchr(buffer, Caracteres[i]);

    if (C != NULL) {                //Si el caracter se encuentra, ocurre lo siguiente;
      ValRec[i] = atof(C + 1);  //Convertir el valor de la variable buscada a flotante
      Presencia[i] = true;      //Se marca que dicho valor esta presente.
    }

    else {
      ValRec[i] = 0.0; //Si no se encuentra el caracter, se pone en 0 el valor numerico en la memoria y se indica que el valor no estaba presente
      Presencia[i] = false;
    }
  }
}

int Signo(float numero) {
  if (numero > 0) {
    return 1;
  } else if (numero < 0) {
    return -1;
  } else {
    return 0;
  }
}

void dibujaLinea(float x, float y, float z,float Velocidad) { //La funcion recibe como argumento la posicion de destino del cabezal y la velocidad a la que debe aproximarse.

  float VectorDestino[3]={0,0,0}; //Calculamos el vector deireccion que apunta a la posicion de destino (X,Y,Z). 
  float VectorPasos[3]={0,0,0};   //Aqui se almacenan los pasos para ir a la posicion de destino (X,Y,Z).
  float RPM[3]={0,0,0}; //Guardamos las rpm de los motores de cada eje
  
  VectorDestino[0]= x - XYZ[0];
  VectorDestino[1]= y - XYZ[1];
  VectorDestino[2]= z - XYZ[2];

  float InvMag = 1 / sqrt(pow(VectorDestino[0], 2) + pow(VectorDestino[1], 2) + pow(VectorDestino[2], 2));

  for (int i = 0; i < 3; i++) {
    VectorPasos[i] = VectorDestino[i] * StpPormm[i]; //CAlculamos los pasos en cada coordenada XYZ
  }

  for (int i = 0; i < 3; i++) {
    VectorDestino[i] = VectorDestino[i] * InvMag; //Normalizamos el vector direccion.
  }

  for (int i = 0; i < 3; i++) {
    VectorDestino[i] = VectorDestino[i] * Velocidad; //Lo multiplicamos por la velocidad para saber la velocidad necesaria en cada eje XYZ
  }

  for (int i = 0; i < 3; i++) {
    RPM[i]=(60*abs(VectorDestino[i])*StpPormm[i])/StpsPRev[i]; //Lo convertimos a RPM para luego enviarlo a los motores.
  }
   
  EjeX.SetSpeed(RPM[0]); //
  EjeY.SetSpeed(RPM[1]); //Establece la velocidad de los motores
  EjeZ.SetSpeed(RPM[2]); //

  int PasoX = 0; //
  int PasoY = 0; // Estas variables indican el paso actual por el que va.
  int PasoZ = 0; //

  int PasoAnteriorX = 0; //
  int PasoAnteriorY = 0; // Estas sirven para comparar  con la variable anterior y ver si hubo un incremento.
  int PasoAnteriorZ = 0; //


  //Se deja la recta en funcion de X
  if ((abs(x) >= abs(y)) & (abs(x) >= abs(z)) & (VectorPasos[0] != 0)) {

    float dy = VectorPasos[1] / abs(VectorPasos[0]); //Delta Y, Es el incremento del eje Y respecto al eje X  ||
    float dz = VectorPasos[2] / abs(VectorPasos[0]); //Delta Z, Es el incremento del eje Z respecto al eje X

    int direccionX = Signo(VectorPasos[0]); //
    int direccionY = Signo(dy);  // Estas variables guardan la direccion de avance de cada eje, siendo 1 si avanza en el eje o -1 si retrocede
    int direccionZ = Signo(dz);  //

    for (int i = 0; PasoX <= abs(VectorPasos[0]); PasoX++) {

      PasoY = round(PasoX * abs(dy));
      PasoZ = round(PasoX * abs(dz));

      if (PasoZ > PasoAnteriorZ) {

        EjeZ.Step(direccionZ * (PasoZ - PasoAnteriorZ));
        PasoAnteriorZ = PasoZ;
      }
      if (PasoY > PasoAnteriorY) {

        EjeY.Step(direccionY * (PasoY - PasoAnteriorY));
        PasoAnteriorY = PasoY;
      }

      if (PasoX > PasoAnteriorX) {

        EjeX.Step(direccionX);
        PasoAnteriorX = PasoX;
      }

    }

  }

  //Se deja la recta en funcion de Y
  else if ((abs(y) >= abs(x)) & (abs(y) >= abs(z)) & (VectorPasos[1] != 0)) {

    float dx = VectorPasos[0] / abs(VectorPasos[1]); //Delta X, Es el incremento del eje X respecto al eje Y  ||
    float dz = VectorPasos[2] / abs(VectorPasos[1]); //Delta Z, Es el incremento del eje Z respecto al eje Y

    int direccionX = Signo(dx);   //
    int direccionY = Signo(VectorPasos[1]);  // Estas variables guardan la direccion de avance de cada eje, siendo 1 si avanza en el eje o -1 si retrocede
    int direccionZ = Signo(dz);   //

    for (int i = 0; PasoY <= abs(VectorPasos[1]); PasoY++) {

      PasoX = round(PasoY * abs(dx));
      PasoZ = round(PasoY * abs(dz));

      if (PasoX > PasoAnteriorX) {

        EjeX.Step(direccionX * (PasoX - PasoAnteriorX));
        PasoAnteriorX = PasoX;
      }

      if (PasoZ > PasoAnteriorZ) {

        EjeZ.Step(direccionZ * (PasoZ - PasoAnteriorZ));
        PasoAnteriorZ = PasoZ;
      }

      if (PasoY > PasoAnteriorY) {

        EjeY.Step(direccionY);
        PasoAnteriorY = PasoY;
      }

    }

  }

  //Se deja la recta en funcion de Z
  else if ((abs(z) >= abs(x)) & (abs(z) >= abs(y)) & (VectorPasos[2] != 0)) {

    float dx = VectorPasos[0] / abs(VectorPasos[2]); //Delta X, Es el incremento del eje X respecto al eje Z  ||
    float dy = VectorPasos[1] / abs(VectorPasos[2]); //Delta Z, Es el incremento del eje Z respecto al eje Z

    int direccionX = Signo(dx);  //
    int direccionY = Signo(dy);  // Estas variables guardan la direccion de avance de cada eje, siendo 1 si avanza en el eje o -1 si retrocede
    int direccionZ = Signo(VectorPasos[2]); //

    for (int i = 0; PasoZ <= abs(VectorPasos[2]); PasoZ++) {

      PasoX = round(PasoZ * abs(dx));
      PasoY = round(PasoZ * abs(dy));

      if (PasoX > PasoAnteriorX) {

        EjeX.Step(direccionX * (PasoX - PasoAnteriorX));
        PasoAnteriorX = PasoX;
      }

      if (PasoY > PasoAnteriorY) {

        EjeY.Step(direccionY * (PasoY - PasoAnteriorY));
        PasoAnteriorY = PasoY;
      }

      if (PasoZ > PasoAnteriorZ) {

        EjeZ.Step(direccionZ);
        PasoAnteriorZ = PasoZ;
      }

    }

  }

  XYZ[0] = (round(VectorPasos[0]) / StpPormm[0]) + XYZ[0]; //
  XYZ[1] = (round(VectorPasos[1]) / StpPormm[1]) + XYZ[1]; //Actualizamos en la memoria la nueva posicion actual
  XYZ[2] = (round(VectorPasos[2]) / StpPormm[2]) + XYZ[2]; //

}

bool MovimientoLineal(int tipo) { //Esta funcion automatiza el procesar los parametros recibidos, tanto para el comando G00 y G01 para enviarlos a la funcion dibujaLinea

  int VelDefault=0;
  
  if(tipo==0){
    VelDefault=40;
  }
  else{
    VelDefault=20;
  }
  
  procesamiento_de_entrada(); //Procesa los valores de entrada que estan el el buffer del puerto serie

  if (Presencia[0] || Presencia[1] || Presencia[2]) { //Comprueba si se ha recibido al menos una coordenada en algun eje (ordenados como x,y,z).

    float ParametrosDeArgumento[4] = {0, 0, 0, 0}; //En este array se guardaran las coordenadas XYZ que se pasaran como paramtro a la funcion dibujaLinea.

    for (int i = 0; i < 3; i++) { //Ejecuta el bucle 3 veces, una por cada coordenada
      if (Presencia[i]) {
        ParametrosDeArgumento[i] = ValRec[i]; //Si Presencia guarda verdadero, se pasa el valor de la coordenada guardado en la memoria del buffer al array que se usara mas tarde
      }
      else {
        ParametrosDeArgumento[i] = XYZ[i]; // Si guarda falso, se pasa el valor del array que codifica la posicion actual del cabezal
      }
    }
    if (Presencia[6]){ //Comprobamos si enviaron el parametro F
    
      ParametrosDeArgumento[3]=ValRec[6];// Si es asi, lo usmamos para seleccionar la velocidad de avance
    }

    else{
      ParametrosDeArgumento[3]=VelDefault; //Caso contrario usamos la velocidad por defecto (40mm/s)
    }
    
    dibujaLinea(ParametrosDeArgumento[0], ParametrosDeArgumento[1], ParametrosDeArgumento[2],ParametrosDeArgumento[3]); //Una vez seleccionado los valores de xyz que se van a pasar, finalmente se lo envian a la funcion.
    return true; //Se devuelve verdadero una vez que la instruccion se ha realizado.
  }

  else {

    return false; //Si no se ha recibido ningun parametro o coordenada, se retorna falso y no se hace nada.
  }
}

void G0(){
    bool stat=MovimientoLineal(0);
   
    if(stat){
      
      Serial.println("Comando G0 recibido y ejecutado");
      Serial.print("X: ");
      Serial.println(XYZ[0]);
      Serial.print("Y: ");
      Serial.println(XYZ[1]);
      Serial.print("Z: ");
     Serial.println(XYZ[2]);
    }
    else{
      
      Serial.println("Error, Parametros insuficientes"); 
      Serial.println("Reenvie el comando");
    }
    
}

void G1() {
    bool stat=MovimientoLineal(1);
   
    if(stat){
      
      Serial.println("Comando G1 recibido y ejecutado");
      Serial.print("X: ");
      Serial.println(XYZ[0]);
      Serial.print("Y: ");
      Serial.println(XYZ[1]);
      Serial.print("Z: ");
     Serial.println(XYZ[2]);
    }
    else{
      
      Serial.println("Error, Parametros insuficientes"); 
      Serial.println("Reenvie el comando");
    }
    
}

void G2() {
  procesamiento_de_entrada();
  // Código específico para el comando G2
  // Aquí puedes agregar la lógica que deseas ejecutar para el comando G2
  Serial.println("Comando G2 recibido");
}

void G3() {
  procesamiento_de_entrada();
  // Código específico para el comando G3
  // Aquí puedes agregar la lógica que deseas ejecutar para el comando G3
  Serial.println("Comando G3 recibido");
}

void G4() {
  procesamiento_de_entrada();

  if(Presencia[9]){

    delay(ValRec[9]);
  }
  else{
    delay(200);
  }
  Serial.println("Comando G4 recibido");
}

void G20() {
  
  Serial.println("No trabajo en pulgadas, no soy yankee. Mejor deja de usar un sistema tan inconveniente solo para sentirte especial");
}

void G21() {
  
  Serial.println("Yo ya trabajo en milimetros, pero ahora tembien se que somos bros de corazon. Arriba el Sistema internacional de unidades XD");
}

void G28() {

  dibujaLinea(XYZ[0],XYZ[1],XYZ[2]+ElevaZ,40);
  dibujaLinea(0,0,XYZ[2],40);
  dibujaLinea(0,0,0,40);
  Serial.println("Se ha enviado el cabezal al origen");
}

void G30() {

}

void G90() {

}

void G91() {

}

void G92() {

}


void setup() {
  Serial.begin(9600); // Inicializar el puerto serie con una velocidad de 9600 bps
  InitSR(2, 3, 4, 2); //Esta funcion es secreta :)
}

void loop() {



  if (Serial.available() > 0) {
    int bytesRead = Serial.readBytesUntil('\n', buffer, bufferSize); // Leer hasta que se reciba el caracter de nueva línea ('\n')
    buffer[bytesRead] = '\0'; // Agregar el caracter nulo al final del buffer

    // Verificar el comando recibido

    if (strncmp(buffer, "G20", 3) == 0)
    {
      G20();
    }

     else if (strncmp(buffer, "G21", 3) == 0)
    {
      G21();
    }
    
    else if (strncmp(buffer, "G01", 3) == 0 || strncmp(buffer, "G1", 2) == 0)
    {
      G1();
    }

    else if (strncmp(buffer, "G02", 3) == 0 || strncmp(buffer, "G2", 2) == 0)
    {
      G2();
    }

    else if (strncmp(buffer, "G03", 3) == 0 || strncmp(buffer, "G3", 2) == 0)
    {
      G3();
    }

    else if (strncmp(buffer, "G04", 3) == 0 || strncmp(buffer, "G4", 2) == 0)
    {
      G4();
    }

    else if (strncmp(buffer, "G00", 3) == 0 || strncmp(buffer, "G0", 2) == 0)
    {
      G0();
    }

  }

  // Resto del código
}

# Este repositorio se utilizará durante la cursada de SOA

El repositorio esta conformado por tres directorios, los alumnos deberán almacenar lo desarrollado en sus actividades prácticas. A continuación se espécifica el contenido que debe tener cada directorio.

- **ANDROID:** En este directorio se debe colocar el código fuente del proyecto desarrollado en Android Studio. 
- **EMBEBIDO:** En este directorio se debe colocar el código fuente del proyecto del embebido desarrollado(ESP32, Arduino,Raspberry PI, etc.).
- **INFORMES:** En este directorio se deben colocar los informes realizados en las actividades prácticas. También aquí se puede subir material complementario de documentación del proyecto (Diagramas,imagenes, videos,entre otros)
---
# AviFeeder: Pajarera inteligente
AviFeeder es una solución tecnológica inclusiva compuesta por una pajarera inteligente y una aplicación móvil, diseñada para mejorar el mantenimiento del equipo y conocimiento de su estado de forma remota.

## Integrantes del Proyecto

| Nombre completo                  | DNI       |
|----------------------------------|-----------|
| BURNOWICZ, ALEJO                 | 42646860  |
| CORREA, JUAN PABLO               | 40653000  |
| FEDERICO, AGUSTIN                | 41882938  |
| MATELLAN, GONZALO FACUNDO        | 43325268  |
| REPETTO, NAHUEL BAUTISTA         | 42024337  |

## Descripción General

El sistema AviFeeder se compone de dos elementos principales:

-	**AviFeeder (Hardware)**: Pajarera inteligente con sensores de detección del contenido de los contenedores de alimento y agua, y sistemas de alerta multisensorial.
-	**AviFeeder App (Software)**: Aplicación móvil Android que permite el control remoto y monitoreo en tiempo real para el usuario.

## Características de la Pajarera

-	Detección de alimento en el contenedor mediante sensor ultrasónico.
-	Sensor de agua para identificar si el contendor de agua está vacío.
-	Alerta Unicanal: Luces LEDs que no afecten el comportamiento de las aves.
-	Controles remotos:
  -	 Botones de controles de LEDs: Encendido/apagado de cada LED de forma independiente.
  -	 Mecanismo de reinicio de sensores: Reinicio del sistema de detección vía sensores por medio de agitación del dispositivo Android.
-	Basado en ESP32 con FreeRTOS.

## Funcionamiento General

1. Al encender el bastón, todos los sensores y sistemas quedan habilitados.
2. Se detectan obstáculos en tres direcciones: izquierda, derecha y frente.
3. Se alerta al usuario mediante:
   - Buzzer (con tonos distintos según cercanía)
   - Motor vibrador (patrones distintos según dirección)
   - Luces LED (verde para sistema activo, rojo para caída)
4. Si el usuario suelta el bastón:
   -	Se activa una alerta (buzzer + LED rojo).
   -	Se notifica automáticamente a la aplicación SmartCane App con la ubicación del usuario.

## Descripción de los Estados del Sistema AviFeeder

| Estado                  | Descripción                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `INIT`               | Estado inicial. Enciende el dispositivo y lo conecta a red WiFi |
| `SERVO_OPEN`            | Estado activo del sistema, recibe y atiende eventos del sistema mientras el servomotor está abierto para carga de alimento |
| `SERVO_CLOSE` | Estado activo del sistema, recibe y atiende eventos del sistema mientras el servomotor está cerrado para evitar carga de alimento excesiva |
	
## AviFeeder App - Aplicación Móvil

Una app Android complementaria para el usuario:
-	Encender/apagar LEDs de forma remota.
-	Recibir actualización sobre el estado de los contenedores.
-	Visualización del tiempo en ejecución

La app utiliza el protocolo MQTT para comunicación en tiempo real entre la pajarera, el servidor y el smartphone.

## Tecnologías Utilizadas

-	**ESP32 + FreeRTOS**
-	**Sensores:** Sensor de distancia, Potenciómetro, Sensor de agua
-	**Actuadores:** Servomotor, LEDs
-	**MQTT** (broker para comunicación IoT)
-	**Android** (Java)

# Prototipo realizado
![AviFeeder Prototipo](https://github.com/UNLAM-SOA/2025-SOA-Q2-M4/blob/main/Prototipo%20Pajarera.png)

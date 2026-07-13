# TMS570 ADC2CAN Rear ECU

Firmware para la ECU ADC2CAN trasera basada en TI TMS570LS1224. El proyecto toma senales analogicas, aplica conversiones y calibraciones, y publica los datos por CAN para el resto de la red del auto.

El punto de entrada activo es [source/sys_main.c](source/sys_main.c).

## CAN

Todos los frames de datos usan 8 bytes (salvo calibracion). El orden de bytes en las tablas es el orden en el bus: b0 es el primer byte y b7 el ultimo.

### Message boxes

| Box | Tipo | CAN ID | Estructura |
| --- | --- | ---: | --- |
| 1 | RX | `0x600` | `calibration_cmd_t` |
| 2 | TX | `0x503` | `current_data_t` |
| 3 | TX | `0x504` | `rear_data_t` |

### Frame recibido

| CAN ID | Estructura | Bytes |
| --- | --- | --- |
| `0x600` | `calibration_cmd_t` | `b0 = cmd_id`, `b1-b7 = libre` |

### Frames transmitidos

| CAN ID | Estructura | Bytes |
| --- | --- | --- |
| `0x503` | `current_data_t` | `b0-b1 = ac_drv1_current_rms`, `b2-b3 = ac_drv2_current_rms`, `b4-b5 = dc_drv1_current_rms`, `b6-b7 = dc_drv2_current_rms` |
| `0x504` | `rear_data_t` | `b0-b1 = rear_brake_pressure`, `b2-b7 = libre` |

### Frame de calibracion (ID `0x600`)

Condicion de seguridad recomendada:
enviar comandos de calibracion solo con el auto en estado seguro (sin torque habilitado).

`DLC = 1`

| `cmd_id` | Accion |
| ---: | --- |
| `0` | Sin accion |
| `1` | Calibracion TPS 0% |
| `2` | Calibracion TPS 100% |
| `3` | Calibracion izquierda volante |
| `4` | Calibracion centro volante |
| `5` | Calibracion derecha volante |
| `6` | Calibracion sensores de corriente |

## ADC2CAN Trasera

### `0x503`: Current Data

`DLC = 8`

| Bytes | Tipo | Significado |
| --- | --- | --- |
| `B0 (LSB), B1 (MSB)` | `uint16_t` | AC Driver 1 Current RMS |
| `B2 (LSB), B3 (MSB)` | `uint16_t` | AC Driver 2 Current RMS |
| `B4 (LSB), B5 (MSB)` | `uint16_t` | DC Driver 1 Current RMS |
| `B6 (LSB), B7 (MSB)` | `uint16_t` | DC Driver 2 Current RMS |

### `0x504`: Rear Data

`DLC = 8`

| Bytes | Tipo | Significado |
| --- | --- | --- |
| `B0 (LSB), B1 (MSB)` | `uint16_t` | Rear Brake Pressure |
| `B2 - B7` | - | Free |

## Flujo de ejecucion

- Inicializacion en [source/sys_main.c](source/sys_main.c): `sciInit`, `canInit`, `init_sensors`.
- Procesamiento periodico por RTI compare2: `convert_data` y `process_calibration_command`.
- Envio periodico por RTI compare3: `send_data_to_serial` y `send_data_to_can`.
- Recepcion CAN en [source/sys_main.c](source/sys_main.c): `canMessageNotification` para mailbox 1 (`0x600`).

## Variables y conversiones

- Sensores y calibraciones en [include/sensors.h](include/sensors.h): `sensors_data_t`, `calibration_cmd_id_t`.
- Logica de conversion y empaquetado CAN en [source/sensors.c](source/sensors.c).
- Conversores activos:
	- Corriente RMS de 4 canales: `ac_drv1`, `ac_drv2`, `dc_drv1`, `dc_drv2`.
	- Presion de freno trasero en PSI (segun ecuacion del sensor).

## Resumen corto

- `0x600` (RX, DLC 1): comando de calibracion.
- `0x503` (TX, DLC 8): corrientes RMS de los cuatro canales.
- `0x504` (TX, DLC 8): presion de freno trasero.
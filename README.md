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
| 4 | TX | `0x401` | implausibility |

### Frame recibido

| CAN ID | Estructura | Bytes |
| --- | --- | --- |
| `0x600` | `calibration_cmd_t` | `b0 = cmd_id`, `b1-b7 = libre` |

### Frames transmitidos

| CAN ID | Estructura | Bytes |
| --- | --- | --- |
| `0x503` | `current_data_t` | `b0-b1 = ac_drv1_current_rms`, `b2-b3 = ac_drv2_current_rms`, `b4-b5 = dc_drv1_current_rms`, `b6-b7 = dc_drv2_current_rms` |
| `0x504` | `rear_data_t` | `b0-b1 = rear_brake_pressure`, `b2-b7 = libre` |
| `0x401` | implausibility | `b0 = TPS implausibility (siempre 0)`, `b1 = brake implausibility`, `b2-b7 = 0` |

### Frame de calibracion (ID `0x600`)

Condicion de seguridad recomendada:
enviar comandos de calibracion solo con el auto en estado seguro (sin torque habilitado).

`DLC = 1`

| `cmd_id` | Accion en esta placa |
| ---: | --- |
| `0` | Sin accion |
| `1` | Calibracion freno trasero a 0 PSI (`calibration_brake_value`) |
| `2` - `5` | Ignorados (TPS y volante, los hace la delantera) |
| `6` | Calibracion (cero) de los cuatro sensores de corriente |
| `7` - `0xE` | Ignorados: son comandos de la main ECU (drive, traccion, modo, telemetria) |

Solo los comandos `1` y `6` escriben la calibracion en la FEE; el resto se descarta sin tocar la flash.

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

### `0x401`: Implausibility

`DLC = 8`. Evento, no periodico: se manda una sola vez cuando se confirma la falla.

| Bytes | Tipo | Significado |
| --- | --- | --- |
| `B0` | `uint8_t` | `1` = TPS implausibility (`error_TPS_IMPLAUSIBILITY`). Siempre `0` en la trasera |
| `B1` | `uint8_t` | `1` = brake implausibility (`ERROR_BREAK_IMPLAUSIBILITY`) |
| `B2 - B7` | - | `0x00` |

Deteccion en la trasera (reglamento T.4.3.3/4, BSE):

- Falla si `rear_brake_raw_value` queda por debajo de 0.4 V (`BRAKE_RAW_MIN = 328`, circuito abierto) o por encima de 4.6 V (`BRAKE_RAW_MAX = 3767`, corto), ADC 12 bits sobre 5 V, durante mas de `IMPLAUSIBILITY_TIME_MS = 100 ms`.
- Los limites son fijos: la calibracion 0 PSI (cmd `1` por `0x600`) solo se usa para convertir a PSI.
- Se mira el valor raw (filtrado por la EMA del ADC): el valor en PSI esta clampeado a 0 y no permite ver un circuito abierto.
- La persistencia se cuenta en ciclos de `convert_data()` (`DATA_PROCESS_HZ = 150 Hz`, `IMPLAUSIBILITY_CYCLES = 15`). Si la condicion desaparece antes, el contador se resetea.
- Por flanco: despues de enviar el frame la placa queda enganchada y se vuelve a armar recien cuando el valor vuelve al rango normal.
- Si el mailbox esta ocupado, el envio se reintenta en el ciclo siguiente.
- La main ECU, al recibir `B0 != 0` o `B1 != 0`, fuerza drive off (igual que el cmd 8 del `0x600`). No guarda la falla: el auto vuelve a andar con un drive-on nuevo (cmd 7 por `0x600`).

## Flujo de ejecucion

- Inicializacion en [source/sys_main.c](source/sys_main.c): `sciInit`, `canInit`, `init_sensors`.
- Procesamiento periodico por RTI compare2: `convert_data` (incluye deteccion de implausibilidad y envio del `0x401`) y `process_calibration_command`.
- Envio periodico por RTI compare3: `send_data_to_serial` y `send_data_to_can`.
- Recepcion CAN en [source/sys_main.c](source/sys_main.c): `canMessageNotification` para mailbox 1 (`0x600`).

## Variables y conversiones

- Sensores y calibraciones en [include/sensors.h](include/sensors.h): `sensors_data_t`, `calibration_cmd_id_t`.
- Logica de conversion y empaquetado CAN en [source/sensors.c](source/sensors.c).
- Conversores activos:
	- Corriente RMS de 4 canales: `ac_drv1`, `ac_drv2`, `dc_drv1`, `dc_drv2`.
	- Presion de freno trasero en PSI (segun ecuacion del sensor).

## Watchdog

- Watchdog digital del RTI (DWD) en [source/sys_main.c](source/sys_main.c), con reset del micro al vencer.
- Vence en ~100 ms (`WATCHDOG_DWD_PRELOAD` 243, con RTICLK = VCLK = 20 MHz). Si se cambia el clock en HALCoGen hay que recalcularlo.
- El lazo principal lo refresca solo si desde el ultimo refresco corrio la interrupcion de muestreo de corrientes (compare 0) y el propio lazo hizo el procesamiento y el envio. Si una interrupcion o el lazo se cuelgan, o el RTI deja de interrumpir, la placa se resetea.
- Los bucles de espera de la FEE en [source/sensors.c](source/sensors.c) tambien lo refrescan, porque un borrado o una escritura de flash pueden durar mas que el watchdog.
- Una vez habilitado no se puede apagar hasta el proximo reset. Para depurar con breakpoints, si el halt resetea la placa, compilar con `WATCHDOG_ENABLE 0`.

## Resumen corto

- `0x600` (RX, DLC 1): comando de calibracion.
- `0x503` (TX, DLC 8): corrientes RMS de los cuatro canales.
- `0x504` (TX, DLC 8): presion de freno trasero.
- `0x401` (TX, DLC 8, por evento): implausibilidad de freno trasero (`b1 = 1`).
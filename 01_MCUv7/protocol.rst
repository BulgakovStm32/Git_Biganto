Протокол обмена с MCU
=====================

Для MCU6

Формат транзакций
-----------------

Все транзакции используют протокол SMBus Block Write - Block Read Process Call:

S Addr+Wr[A] Comm[A] Count[A] Data1[A] Data2[A] ... DataN[A] aPEC[A] Sr Addr+Rd[A] [Count]A [Data1]A [Data2]A ... [DataN]A [PEC]N P

=============  ===============================================
S     (1 бит)  Старт бит
Sr    (1 бит)  Повторный старт бит
P     (1 бит)  Стоп бит
Rd/Wr (1 бит)  Бит чтение/запись. Чтение - 1, запись - 0.
A, NA (1 бит)  Бит подтверждения.
Addr  (7 бит)  Адрес ведомого.
Comm  (8 бит)  ID команды.
Data  (8 бит)  Байты данных.
Count (8 бит)  Размер блока. Расчитывается как DataN+aPEC.
PEC   (8 бит)  CRC8 для всей транзакции.
[..]           Данные посылаемые ведомым устройством ведущему.
=============  ===============================================


Команды
-------

В блоке данных от хоста каждой команды содержится дополнительный PEC рассчитанный по обычным правилам. Этот PEC не
является частью протокола Block Write - Block Read Process Call, поэтому он считается частью данных. Дополнительный PEC
далее обозначается как aPEC (additional PEC).
Чтобы убедиться в правильности работы ответ от MCU должен содержать ID выполненной команды первым байтом.

==============================  ====  ======  ========================================================
Команда                          ID   Размер  Описание
==============================  ====  ======  ========================================================
GetCurrentPosition              0x00  W1R15   Получить текущий угол
GetCurrentAcceleration          0x01  W1R15   Получить текущее ускорение
GetCurrentVelocity              0x02  W1R15   Получить текущую скорость
SetTargetPosition               0x03  W5R1    Задать целевое положение
SetMaxAcceleration              0x04  W9R1    Задать максимальное ускорение
SetMaxVelocity                  0x05  W5R1    Задать максимальную скорость
SetMicrostep                    0x06  W2R1    Задать микрошаг
ResetTMC                        0x07  W1R1    Сбросить чип
ResetPosition                   0x08  W1R1    Сбросить позицию
EmergencyStop                   0x09  W1R1    Аварийная остановка. Актуальна при застреваниях
ArduinoMicroTS                  0x0A  W1R5    Запросить текущее значение микросекунд таймера Ардуино
ArduinoMeasurePulseCalibration  0x0B  W1R1    Запомнить текущее значение таймера Ардуино
ArduinoGetPulseCalibration      0x0C  W1R9    Получить измеренное значение таймера
EnableStealthChop               0x0D  W2R1    Активировать бесшумный режим
ReadTMC4361Register             0x0E  W2R15   Прочитать значение регистра чипа tmc4361
WriteTMC4361Register            0x0F  W6R1    Задать значение регистра чипа tmc4361
ReadTMC2130Register             0x10  W2R15   Прочитать значение регистра чипа tmc2130
WriteTMC2130Register            0x11  W6R1    Задать значение регистра чипа tmc2130
EnableChopper                   0x12  W2R1
SetEncoderConstant              0x13  W5R1    Установить константу для энкодера
GetEncoderConstant              0x14  W1R15   Получить константу энкодера
GetEncoderPosition              0x15  W1R15   Получить значение энкодера
ResetEncoderPosTolCtrl          0x16  W1R1
StartEncoderPosTolCtrl          0x17  W1R1    Включение системы контроля отклонения положения энкодера
StopEncoderPosTolCtrl           0x18  W1R1
SetEncoderPosTol                0x19  W5R1
GetEncoderPosTol                0x1A  W1R5
StartCtrlRequestTime            0x1B  W1R1
StopCtrlRequestTime             0x1C  W1R1
GetMotorAndEncoderPosition      0x1D  W1R29   Прочитать положение мотора и положение энкодера
GetShadow                       0x20  W2R15   Прочитать теневой регистр
GetStatusAndFlagReg             0x21  W1R5    Получить текущее состояние API
ResetError                      0x22  W1R1    Сбросить ошибки API
PeripheralsPowerCtl             0x27  W2R1    Управление питанием переферии
TurnOffPower                    0x28  W5R1    Выключить питание
GetSystemCtrlReg                0x32  W1R5    Чтение системного регистра управления
GetTemperature                  0x34  W2R5    Получение значения от датчика температуры
CheckSensors                    0x35  W1R1    Наличие температурных датчиков
MeasureTemperature              0x36  W1R1    Начать измерение температуры
ConfigEncoder                   0x37  W3R1    Настроить работу с SSI энкодером
FirmwareVersion                 0x38  W3R18   Получить строку с версией прошивки
Signal                          0x39  W2R1    Отправить сигнал пользователю
==============================  ====  ======  ========================================================


GetCurrentPosition
^^^^^^^^^^^^^^^^^^

| Запрос: 0x00 0x01 [aPEC]
| Ответ : 0x0F 0x00 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


GetCurrentAcceleration
^^^^^^^^^^^^^^^^^^^^^^

| Запрос: 0x01 0x01 [aPEC]
| Ответ : 0x0F 0x01 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


GetCurrentVelocity
^^^^^^^^^^^^^^^^^^

| Запрос: 0x02 0x01 [aPEC]
| Ответ : 0x0F 0x02 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


SetTargetPosition
^^^^^^^^^^^^^^^^^

| Запрос: 0x03 0x05 [1] [aPEC]
| Ответ : 0x01 0x03
| 1 - u32 data


SetMaxAcceleration
^^^^^^^^^^^^^^^^^^

| Запрос: 0x04 0x09 [1] [2] [aPEC]
| Ответ : 0x01 0x04
| 1 - u32 accelerationMax
| 2 - u32 accelerationStart


SetMaxVelocity
^^^^^^^^^^^^^^

| Запрос: 0x05 0x05 [1] [aPEC]
| Ответ : 0x01 0x05
| 1 - u32 data


SetMicrostep
^^^^^^^^^^^^

| Запрос: 0x06 0x02 [1] [aPEC]
| Ответ : 0x01 0x06
| 1 - u8 data


ResetTMC
^^^^^^^^

| Запрос: 0x07 0x01 [aPEC]
| Ответ : 0x01 0x07


ResetPosition
^^^^^^^^^^^^^

| Запрос: 0x08 0x01 [aPEC]
| Ответ : 0x01 0x08


EmergencyStop
^^^^^^^^^^^^^

| Запрос: 0x09 0x01 [aPEC]
| Ответ : 0x01 0x09


ArduinoMicroTS
^^^^^^^^^^^^^^

| Запрос: 0x0A 0x01 [aPEC]
| Ответ : 0x05 0x0A [1]
| 1 - u32 временная отметка


ArduinoMeasurePulseCalibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Запомнить текущее значение таймера Ардуино для последующего извлечения с помощью ArduinoGetPulseCalibration.

| Запрос: 0x0B 0x01 [aPEC]
| Ответ : 0x01 0x0B


ArduinoGetPulseCalibration
^^^^^^^^^^^^^^^^^^^^^^^^^^

Получить измеренное значение таймера с помощью команды ArduinoMeasurePulseCalibration.

| Запрос: 0x0C 0x01 [aPEC]
| Ответ : 0x09 0x0C [1] [2]
| 1 - u32 beginTS
| 2 - u32 endTS


EnableStealthChop
^^^^^^^^^^^^^^^^^

| Запрос: 0x0D 0x02 [1] [aPEC]
| Ответ : 0x01 0x0D
| 1 - u8 enable


ReadTMC4361Register
^^^^^^^^^^^^^^^^^^^

| Запрос: 0x0E 0x02 [2] [aPEC]
| Ответ : 0x0F 0x0E [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


WriteTMC4361Register
^^^^^^^^^^^^^^^^^^^^

| Запрос: 0x0F 0x06 [1] [2] [aPEC]
| Ответ : 0x01 0x0F
| 1 - u8 TMC reg address
| 2 - u32 data


ReadTMC2130Register
^^^^^^^^^^^^^^^^^^^

| Запрос: 0x10 0x02 [2] [aPEC]
| Ответ : 0x0F 0x10 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


WriteTMC2130Register
^^^^^^^^^^^^^^^^^^^^

| Запрос: 0x11 0x06 [1] [2] [aPEC]
| Ответ : 0x01 0x11
| 1 - u8 TMC reg address
| 2 - u32 data


EnableChopper
^^^^^^^^^^^^^

| Запрос: 0x12 0x02 [1] [aPEC]
| Ответ : 0x01 0x12
| 1 - u8 enable


SetEncoderConstant
^^^^^^^^^^^^^^^^^^

| Запрос: 0x13 0x05 [1] [aPEC]
| Ответ : 0x01 0x13
| 1 - u32 data


GetEncoderConstant
^^^^^^^^^^^^^^^^^^

| Запрос: 0x14 0x01 [aPEC]
| Ответ : 0x0F 0x14 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


GetEncoderPosition
^^^^^^^^^^^^^^^^^^

| Запрос: 0x15 0x01 [aPEC]
| Ответ : 0x0F 0x15 [1] [2] [3] [4] [5]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS


ResetEncoderPosTolCtrl
^^^^^^^^^^^^^^^^^^^^^^

Сбросить значение в ноль для системы контроля отклонения энкодера от текущей позиции мотора.

| Запрос: 0x16 0x01 [aPEC]
| Ответ : 0x01 0x16


StartEncoderPosTolCtrl
^^^^^^^^^^^^^^^^^^^^^^

Команда на включение системы контроля отклонения положения энкодера от текущей позиции мотора. Если данная система
включена, то при превышении отклонения положения энкодера выше заданной величины будет установлена соответствующая
ошибка и аварийно остановлен включенный двигатель.

| Запрос: 0x17 0x01 [aPEC]
| Ответ : 0x01 0x17


StopEncoderPosTolCtrl
^^^^^^^^^^^^^^^^^^^^^

Команда на выключение системы контроля отклонения положения энкодера от текущей позиции мотора. Если данная система
включена, то при превышении отклонения положения энкодера выше заданной величины будет установлена соответствующая
ошибка и аварийно остановлен включенный двигатель.

| Запрос: 0x18 0x01 [aPEC]
| Ответ : 0x01 0x18


SetEncoderPosTol
^^^^^^^^^^^^^^^^

Установить зону нечувствительности отклонения положения энкодера от текущей позиции мотора.

| Запрос: 0x19 0x05 [1] [aPEC]
| Ответ : 0x01 0x19
| 1 - u32 data


GetEncoderPosTol
^^^^^^^^^^^^^^^^

Получить величину зоны нечувствительности отклонения положения энкодера от текущей позиции мотора.

| Запрос: 0x1A 0x01 [aPEC]
| Ответ : 0x05 0x1A [1]
| 1 - u32 data


StartCtrlRequestTime
^^^^^^^^^^^^^^^^^^^^

Команда на включение системы контроля времени между запросами от системы верхнего уровня. Если данная система включена,
то  при отсутствии запросов от верхнего уровня в течение заданного времени будет установлена соответствующая ошибка и
аварийно остановлен включенный двигатель.

| Запрос: 0x1B 0x01 [aPEC]
| Ответ : 0x01 0x1B


StopCtrlRequestTime
^^^^^^^^^^^^^^^^^^^

Команда на выключение системы контроля времени между запросами от системы верхнего уровня. Если данная система включена,
то  при отсутствии запросов от верхнего уровня в течение заданного времени будет установлена соответствующая ошибка и
аварийно остановлен включенный двигатель.

| Запрос: 0x1C 0x01 [aPEC]
| Ответ : 0x01 0x1C


GetMotorAndEncoderPosition
^^^^^^^^^^^^^^^^^^^^^^^^^^

| Запрос: 0x1D 0x01 [aPEC]
| Ответ : 0x1D 0x1D [1] [2] [3] [4] [5] [6] [7] [8] [9] [10]
| 1 - u8 SPI status
| 2 - u8 TMC reg address
| 3 - u32 data
| 4 - u32 beginTS
| 5 - u32 endTS
| 6 - u8 SPI status
| 7 - u8 TMC reg address
| 8 - u32 data
| 9 - u32 beginTS
| 10 - u32 endTS


GetShadow
^^^^^^^^^

| Запрос: 0x20 0x02 [1] [aPEC]
| Ответ : 0x0F 0x20 [2] [3] [4] [5] [6]
| 1 - u8 reg index
| 2 - u8 SPI status
| 3 - u8 TMC reg address
| 4 - u32 data
| 5 - u32 beginTS
| 6 - u32 endTS


GetStatusAndFlagReg
^^^^^^^^^^^^^^^^^^^

| Запрос: 0x21 0x01 [aPEC]
| Ответ : 0x05 0x21 [1]
| 1 - u32 data


ResetError
^^^^^^^^^^

| Запрос: 0x22 0x01 [aPEC]
| Ответ : 0x01 0x22


PeripheralsPowerCtl
^^^^^^^^^^^^^^^^^^^

| Запрос: 0x27 0x02 [1] [aPEC]
| Ответ : 0x01 0x27
| 1 - u8 data

Если data != 0, то включает вентилятор, иначе выключает.


TurnOffPower
^^^^^^^^^^^^

| Запрос: 0x28 0x05 [1] [aPEC]
| Ответ : 0x01 0x28
| 1 - u32 data


GetSystemCtrlReg
^^^^^^^^^^^^^^^^

| Запрос: 0x32 0x01 [aPEC]
| Ответ : 0x05 0x32 [1]
| 1 - u32 data


GetTemperature
^^^^^^^^^^^^^^

| Запрос: 0x34 0x02 [1] [aPEC]
| Ответ : 0x05 0x34 [2]
| 1 - u8 sensor_number
| 2 - u32 temperature/sensorsNum


CheckSensors
^^^^^^^^^^^^

| Запрос: 0x35 0x01 [aPEC]
| Ответ : 0x01 0x35


MeasureTemperature
^^^^^^^^^^^^^^^^^^

Команда считывает прошлые измеренные значения температуры и начинает новое измерение.

| Запрос: 0x36 0x01 [aPEC]
| Ответ : 0x01 0x36


ConfigEncoder
^^^^^^^^^^^^^

| Запрос: 0x37 0x03 [1] [2] [aPEC]
| Ответ : 0x01 0x37
| 1 - u8 resolution
| 2 - u8 is_gray


FirmwareVersion
^^^^^^^^^^^^^^^

| Запрос: 0x38 0x01 [aPEC]
| Ответ : 0x12 0x38 [1]
| 1 - С-строка, максимум из 17 символов (включая '\\0')


Signal
^^^^^^

| Запрос: 0x39 0x02 [1] [aPEC]
| Ответ : 0x01 0x39
| 1 - u8 Разновидность сигнала

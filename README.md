  RO
În cadrul acestui proiect veți dezvolta o aplicație bazată pe sistemul de operare de timp
real freeRTOS și diverse componente pentru a gestiona mai multe task-uri.
Aplicația trebuie să citească temperatura folosind un termistor (de preferat NTC 10k) la
fiecare x ms și să convertească valoarea citită. Se va genera un semnal PWM cu factor de
umplere direct proporțional cu valoarea temperaturii citite. De preferat ca acest semnal PWM să
fie trimis la un ventilator (dar se poate conecta și un led la ieșirea respectivă). Aplicația va
suporta și un mod de funcționare manual astfel: la apăsarea unui buton (B1), task-ul care citește
temperatura nu mai trimite comenzi către task-ul care modifică factorul de umplere al semnalului
PWM. În acest mod, factorul de umplere al semnalului PWM este controlat de către un
potențiometru. Dacă se apasă din nou butonul B1, aplicația va reveni la normal.
Suplimentar, se va dezvolta o aplicație desktop (în orice limbaj ales de student) pentru a
comunica cu aplicația realizată pe microcontroler. Aplicația desktop va avea doar rolul de
vizualizare a datelor și nu se vor trimite comenzi către microcontroler. Informațiile pe care
trebuie să le afișați pe portul serial sunt: starea sistemului (mod manual, mod automat),
temperatura citită, factorul de umplere. Suplimentar, puteți afișa și alte informații. Nu este
obligatoriu ca aplicația să conțină și un GUI. Puteți afișa aceste date într-un terminal.

  ENG
In this project, you will develop an application based on the FreeRTOS real-time operating system and various components to manage multiple tasks.
The application must read the temperature using a thermistor (preferably an NTC 10k) every x milliseconds and convert the measured value. A PWM signal will be generated with a duty cycle directly proportional to the measured temperature value.
Ideally, this PWM signal should be sent to a fan (but an LED can also be connected to that output).
The application will also support a manual operating mode, as follows:
when a button (B1) is pressed, the task that reads the temperature no longer sends commands to the task that modifies the PWM signal’s duty cycle. In this mode, the PWM duty cycle is controlled by a potentiometer. If the button B1 is pressed again, the application returns to automatic mode.
Additionally, a desktop application (written in any language chosen by the student) will be developed to communicate with the microcontroller application. The desktop application will only serve the purpose of data visualization and will not send commands to the microcontroller.
The information that must be displayed over the serial port includes:
the system state (manual mode or automatic mode)
the measured temperature
the PWM duty cycle
Optionally, you may display other data as well.
It is not mandatory for the desktop application to have a GUI; the data can be displayed in a terminal window instead.

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

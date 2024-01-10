# Robotik-Projekt-2-Team-Follow


kamera
/dev/video0

knoten hat cv2 object
stream öffnen im konstruktor
timer auf 10 Hz
-> bild auslesen, entzerren, grauwert, marker erkennen und ggf abstand berechnen

rfid:
from mfrc522 import SimpleMFRC522
-> docu
--> Bib installieren noch zu erledigen

Akku: 12V!!

documentation? -> hauptsächlich im code, readme file zum starten, doxygen?, doc mit erklärung zu verfahren und warum, methoden erklären
follow regelung in matlab simulieren -> k faktor begründen im bericht
erklären wie die abstandsmessung funktioniert

Testen: move, turn

 Fragen: 
 Wie kann man erkennen, ob ein Roboter von hinten oder von vorne gesehen wird?
 
 Oder ist das gar nicht notwendig?
 
 warum stürzt der client bei änderung des goals ab?

 Müssen target_distance und target_velocity über einen Publisher kommen?

 kamera bild mit zeitstempel und triggern beim timer

 

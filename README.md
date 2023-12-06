# Robotik-Projekt-2-Team-Follow
Was bekommen wir von der Kamera -> datentyp, topic, import    
Was bekommen wir vom rfid -> datentyp, topic, import      
Setup -> wo genau ist der rfid
Wer reagiert auf die kamera -> client oder server

Wer/Wo schreibt Werte in result, wie Succeed oder Aborted?
--> Im Server result festlegen

ein knoten hat clients und die state machine
clients leben gleichzeitig
result = enum für state machine
spin bis crtl c

kamera
/dev/video0

knoten hat cv2 object
stream öffnen im konstruktor
timer auf 10 Hz
-> bild auslesen, entzerren, grauwert, marker erkennen und ggf abstand berechnen


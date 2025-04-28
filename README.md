
1.2 Sémaphores pour la synchronisation :
6.
Cas 1 : 
taskGive a une priorité plus élevée
Elle s'exécute immédiatement dès qu’elle est prête
Elle peut bloquer taskTake s’il y a compétition
      
Cas 2 : 
taskTake a une priorité plus élevée
Elle prend la main dès que le sémaphore est donné
Moins de délai entre les messages give et take
L'affichage devient plus fluide et sans "retard" apparent

3.1 Gestion du tas :
1.
C’est le tas FreeRTOS, appelé : ucHeap
2.
C’est entièrement géré par FreeRTOS.

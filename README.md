
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
3.
Memoire FLASH : 29.27KB soit 2.86% ; Memoire RAM : 18.79KB soit 5.87%
4.
Les taches sont créées jusqu’à l’arrivée d'une erreur
5.
La taille du tas a été modifiée, en revanche nous ne sommes pas parvenus à afficher la nouvelle mémoire utilisée
3.2 Gestion des piles
4.
On utilise une LED sur la carte qui s'allume lorsque la pile est remplie

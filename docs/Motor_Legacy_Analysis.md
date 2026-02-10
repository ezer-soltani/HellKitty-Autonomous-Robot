# Analyse Comparative : Driver Moteur Legacy vs Cible

**Date :** 8 Décembre 2025
**Source Legacy :** `@raf/firmware/.../moteur.c`
**Cible :** Nouveau driver modulaire `motor_driver`

Ce document analyse le code moteur de l'année précédente pour en extraire les bonnes idées et identifier les pièges à éviter dans la nouvelle implémentation.

---

## 1. Architecture & Modularité

### Code Legacy (`moteur.c`)
*   **Approche :** "Tout-en-un" (Monolithique). Le fichier contient la gestion bas niveau (PWM/Encodeur), la logique métier (Modes Souris/Prédateur), et même l'algorithme PID.
*   **Dépendances :** Fortement couplé aux variables globales `htim1`, `htim3`, `htim4`. Les fonctions appellent directement les handles globaux.
*   **Duplication :** Fonctions séparées pour `forward_r`, `forward_l`, `reverse_r`, `reverse_l`.
    *   *Critique :* Difficile à maintenir. Si on change un timer, il faut changer 4 fonctions.

### Architecture Cible (`motor_driver`)
*   **Approche :** Orientée Objet (Struct `Motor_Handle_t`).
*   **Modularité :** Le driver ne gère QUE le moteur (PWM + Encodeur). Il ne connait pas le PID, ni la stratégie (Souris/Prédateur).
*   **Abstraction :** Une seule fonction `Motor_SetSpeed(handle, percent)` gère le sens (Fwd/Rev) et le moteur (Gauche/Droit) via la configuration stockée dans la structure.

---

## 2. Qualité du Code (PWM & Encodeurs)

### Code Legacy
*   **Contrôle PWM :** Utilise `__HAL_TIM_SET_COMPARE` et `HAL_TIMEx_PWMN_Start/Stop`.
    *   *Observation :* Utilise les sorties complémentaires (CHxN) du Timer 1. C'est spécifique aux drivers moteur "H-Bridge" avancés.
    *   *Bug Potentiel :* Dans `reverse_r`, on voit des lignes commentées et un mélange Start/Stop qui suggère que la mise au point a été laborieuse.
*   **Calcul Vitesse (`calculate_motor_speed`) :**
    *   Utilise des variables `static` locales pour mémoriser la position précédente.
    *   *Problème :* Cette fonction n'est pas réentrante. Si appelée pour LEFT puis RIGHT, les variables statiques `last_position_motorX` sont séparées par des `if`, ce qui est fragile.
    *   *Gestion Overflow :* La soustraction `current - last` est faite sur des `int16_t`. C'est **correct** pour gérer le débordement naturel des timers 16 bits.

### Architecture Cible
*   **Contrôle PWM :** Adapté à notre hardware actuel (ZXBM5210 sur TIM3). Pas de sorties complémentaires "N" utilisées dans le projet actuel `Test_Motor`.
*   **Calcul Vitesse :** Stockera `prev_counter` dans la structure `Motor_Handle_t`, rendant la fonction de calcul parfaitement réentrante et thread-safe.

---

## 3. Gestion du Temps & FreeRTOS

### Code Legacy
*   **Mélange :** Utilise `HAL_Delay` (bloquant) dans certaines boucles, et `vTaskDelay` (FreeRTOS) dans d'autres (`motorcontrol_souris`).
*   **Affichage :** Utilise beaucoup de `printf` dans les fonctions de contrôle.
    *   *Impact :* Ralentit considérablement la boucle de contrôle, ce qui est désastreux pour un PID stable.

### Architecture Cible
*   **Strictement Non-Bloquant :** Aucune fonction `Delay` ni `printf` dans le driver.
*   **Périodicité :** Le calcul de vitesse `Motor_UpdateSpeed` sera appelé par une tâche cyclique précise (`Task_ControlLoop`) et prendra `delta_time` en argument pour être indépendant de la fréquence d'appel.

---

## 4. Asservissement (PID)

### Code Legacy
*   **Implémentation :** Fonction `compute_pid` standard.
*   **Problème PID :** Le terme intégral `integral_error` n'est pas borné (pas d'Anti-Windup).
    *   *Risque :* Si le moteur est bloqué, l'intégrale explose et le moteur part à fond dès qu'il est débloqué.

### Architecture Cible
*   Le PID sera sorti du driver et mis dans `Core/App/pid_controller.c`.
*   Ajout impératif d'un **Anti-Windup** (limitation de la somme intégrale).

---

## 5. Recommandations pour le Nouveau Driver

1.  **Ne pas copier-coller** le code legacy. L'approche est trop différente.
2.  **S'inspirer** de la gestion du débordement 16-bit des encodeurs (c'était correct).
3.  **Ignorer** la gestion spécifique `TIM1_CH1N` (complémentaire) sauf si notre hardware l'exige (le projet actuel utilise TIM3 qui n'a pas de canaux N sortis sur les pins standards de la même manière pour ce driver).
4.  **Séparer** totalement la logique de contrôle (Avancer, Reculer) de la logique de stratégie (Souris, Prédateur).

**Conclusion :** Le driver legacy est un code de prototypage "rapide". Notre objectif est un code de production "industriel".


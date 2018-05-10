# How to run the robot


## Pendant les tests

Pour lancer Minus, il suffit d'exécuter le launchfile principale de Minus qui se situe dans le package **robot_2018**.
Il suffit donc d'effectuer la commande suivante

```bash
roslaunch robot_2018 minus.launch team:=value
```

Si, on veut lancer rviz pour voir la simulation du robot il suffit de rajouter un paramètre.

```bash
roslaunch robot_2018 minus.launch team:=value viz:=true
```

> Par défaut rviz ne se lance pas et si on omet de mettre le paramètre team, la valeur par défaut est inconnu.
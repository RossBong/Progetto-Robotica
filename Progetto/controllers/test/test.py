from controller import Robot

def muovi_robot_un_metro(robot: Robot, velocita: float):
    # Assumiamo che le ruote del robot abbiano un raggio di 0.05 metri.
    raggio_ruota = 0.05

    # Calcoliamo la distanza in termini di rotazioni delle ruote (in radianti).
    # La circonferenza di una ruota è 2 * pi * raggio, quindi 1 metro corrisponde a 1 / (2 * pi * raggio) rotazioni.
    # Convertiamo le rotazioni in radianti moltiplicandole per 2 * pi.
    distanza_in_radianti = 1 / (2 * pi * raggio_ruota) * 2 * pi

    # Otteniamo i motori.
    motore_sinistro = robot.getMotor('left wheel')
    motore_destro = robot.getMotor('right wheel')

    # Impostiamo la posizione target per i motori.
    motore_sinistro.setPosition(distanza_in_radianti)
    motore_destro.setPosition(distanza_in_radianti)

    # Impostiamo la velocità dei motori.
    motore_sinistro.setVelocity(velocita)
    motore_destro.setVelocity(velocita)

# Creiamo un'istanza del robot.
robot = Robot()

# Chiamiamo la funzione per muovere il robot di un metro.
muovi_robot_un_metro(robot, 1.0)

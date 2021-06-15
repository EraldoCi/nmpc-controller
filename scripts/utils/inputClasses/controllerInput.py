from typing import NamedTuple

class ControllerInput(NamedTuple):
    """Entrada do NMPC.

    Attributes:
        xref: x de referência (Valor recebido da trajatória).
        yref: y de referência (Valor recebido da trajatória).
        RstateX: estado do robo em X.
        RstateY: estado do robo em X.
        RstateTheta: estado do robo em Theta.
        RstateVelocity: estado da velocidade linear do robô.
        RstateW: esdado da velocidade angular do robô.
        xrefA: x de referência anterior.
        yrefA: y de referência anterior.
        thetarefA: Theta de referência anterior.
        vrefA: velocidade linear de referência anterior.
        wrefA: velocidade angular de referência anterior.
    """
    xref: float
    yref: float
    RstateX: float
    RstateY: float
    RstateTheta: float
    RstateVelocity: float
    RstateW: float
    xrefA: float
    yrefA: float
    thetarefA: float
    vrefA: float
    wrefA: float
from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, List
from enum import Enum


class BallPosition(Enum):
    """Posição da bola na visão do robô."""
    NONE = "none"
    CENTER = "Center"
    LEFT_TOP = "Left Top"
    LEFT_BOTTOM = "Left Bottom"
    RIGHT_TOP = "Right Top"
    RIGHT_BOTTOM = "Right Bottom"


class FallState(Enum):
    """Estado de queda do robô."""
    UP = "Up"
    FRONT = "Front"
    BACK = "Back"
    LEFT = "Left"
    RIGHT = "Right"


class HeadHorizontalPosition(Enum):
    """Posição horizontal da cabeça (motor horizontal)."""
    CENTER = "Center"
    LEFT = "Left"
    RIGHT = "Right"


@dataclass
class WorldState:
    ball_angle: float = 0.0
    ball_position: BallPosition = BallPosition.NONE
    ball_close: bool = False
    ball_found: bool = False

    fall_state: FallState = FallState.UP
    recovering: bool = False

    hor_motor_out_of_center: HeadHorizontalPosition = HeadHorizontalPosition.CENTER
    head_kick_check: bool = False
    kick_done: bool = False
    trunk_aligned: bool = False


class StackElement(ABC):
    def __init__(self, name: str):
        self.name = name

    def on_enter(self, world: WorldState) -> None:
        pass

    def on_exit(self, world: WorldState) -> None:
        pass

class DecisionElement(StackElement):
    @abstractmethod
    def perform(self, world: WorldState) -> str:
        pass

    def get_reevaluate(self) -> bool:
        return True


class ActionElement(StackElement):
    @abstractmethod
    def perform(self, world: WorldState) -> str:
        pass

    def do_not_reevaluate(self) -> bool:
        return False


class RootDecision(DecisionElement):
    """Decisão raiz: segurança primeiro."""

    def __init__(self):
        super().__init__("RootDecision")

    def perform(self, world: WorldState) -> str:
        if world.fall_state != FallState.UP or world.recovering:
            return "RECOVERY"

        return "STANDING_OK"


class BallFoundDecision(DecisionElement):
    """Verifica se a visão está encontrando a bola (ball_found)."""

    def __init__(self):
        super().__init__("BallFoundDecision")

    def perform(self, world: WorldState) -> str:
        return "YES" if world.ball_found else "NO"


class BallCloseDecision(DecisionElement):
    """Verifica se a bola está perto (ball_close)."""

    def __init__(self):
        super().__init__("BallCloseDecision")

    def perform(self, world: WorldState) -> str:
        return "YES" if world.ball_close else "NO"


class BallPositionDecision(DecisionElement):
    """Verifica a posição da bola na visão (ball_position)."""

    def __init__(self):
        super().__init__("BallPositionDecision")

    def perform(self, world: WorldState) -> str:
        return world.ball_position.value


class HeadKickCheckDecision(DecisionElement):
    """Verifica se a cabeça está numa inclinação adequada para chutar."""

    def __init__(self):
        super().__init__("HeadKickCheckDecision")

    def perform(self, world: WorldState) -> str:
        return "YES" if world.head_kick_check else "NO"


class HorMotorDecision(DecisionElement):
    """Verifica a posição horizontal da cabeça."""

    def __init__(self):
        super().__init__("HorMotorDecision")

    def perform(self, world: WorldState) -> str:
        return world.hor_motor_out_of_center.value


class StandUpAction(ActionElement):
    """Executa o levantamento após a recuperação liberar o robô."""

    def __init__(self):
        super().__init__("StandUpAction")

    def perform(self, world: WorldState) -> str:
        print(f"Ação: levantando o robô... | fall_state={world.fall_state.value}")

        if world.fall_state != FallState.UP:
            print("Ainda não pode concluir levantamento: robô segue caído.")
            return "RUNNING"

        print("Robô em pé.")
        return "DONE"


class RecoveryAction(ActionElement):
    """Mantém o robô bloqueado até sair da condição de queda."""

    def __init__(self):
        super().__init__("RecoveryAction")

    def perform(self, world: WorldState) -> str:
        print(f"Estado de recuperação | fall_state={world.fall_state.value}")

        if world.fall_state != FallState.UP:
            world.recovering = True
            print("Robô caiu. Locomoção bloqueada.")
            return "RUNNING"

        if world.recovering:
            world.recovering = False
            print("Recuperação concluída.")
            return "DONE"

        return "RUNNING"


class SearchBallAction(ActionElement):
    """Procura a bola girando com base em hor_motor_out_of_center."""

    def __init__(self, angular_vel: float = 0.4):
        super().__init__("SearchBallAction")
        self.angular_vel = angular_vel

    def perform(self, world: WorldState) -> str:
        if world.fall_state != FallState.UP or world.recovering:
            print("Locomoção bloqueada na busca")
            return "DONE"

        if world.hor_motor_out_of_center == HeadHorizontalPosition.LEFT:
            angular_vel = self.angular_vel
        elif world.hor_motor_out_of_center == HeadHorizontalPosition.RIGHT:
            angular_vel = -self.angular_vel
        else:
            angular_vel = self.angular_vel

        print(f"Procurando bola | head={world.hor_motor_out_of_center.value} | w={angular_vel:.2f}")

        return "DONE" if world.ball_found else "RUNNING"


class GoToBallAction(ActionElement):
    """Move o robô em direção à bola usando ball_position para ajuste direcional."""

    def __init__(self, linear_vel: float = 0.3, kp: float = 1.5):
        super().__init__("GoToBallAction")
        self.linear_vel = linear_vel
        self.kp = kp

    def perform(self, world: WorldState) -> str:
        if world.fall_state != FallState.UP or world.recovering:
            print("Locomoção bloqueada na aproximação")
            return "DONE"

        linear_vel = self.linear_vel

        if world.ball_position in [BallPosition.LEFT_TOP, BallPosition.LEFT_BOTTOM]:
            angular_vel = 0.3
        elif world.ball_position in [BallPosition.RIGHT_TOP, BallPosition.RIGHT_BOTTOM]:
            angular_vel = -0.3
        else:
            angular_vel = -self.kp * world.ball_angle

        print(
            f"Indo até a bola | pos={world.ball_position.value} | "
            f"ang={world.ball_angle:.2f} | v={linear_vel:.2f} | w={angular_vel:.2f}"
        )

        if not world.ball_found:
            print("Bola perdida.")
            return "DONE"

        if world.ball_close:
            print("Bola perto.")
            return "DONE"

        return "RUNNING"


class AlignHeadForKickAction(ActionElement):
    """Alinha a cabeça para uma boa posição de chute."""

    def __init__(self):
        super().__init__("AlignHeadForKickAction")

    def perform(self, world: WorldState) -> str:
        if world.fall_state != FallState.UP or world.recovering:
            print("Alinhamento interrompido por segurança.")
            return "DONE"

        print(f"Alinhando cabeça | head_kick_check={world.head_kick_check}")
        return "DONE" if world.head_kick_check else "RUNNING"


class TrunkAlignedDecision(DecisionElement):
    """Verifica se o tronco está alinhado com a bola."""

    def __init__(self):
        super().__init__("TrunkAlignedDecision")

    def perform(self, world: WorldState) -> str:
        return "YES" if world.trunk_aligned else "NO"


class AlignTrunkAction(ActionElement):
    """Alinha o tronco do robô com a bola antes do chute."""

    def __init__(self):
        super().__init__("AlignTrunkAction")

    def perform(self, world: WorldState) -> str:
        if world.fall_state != FallState.UP or world.recovering:
            print("Alinhamento de tronco interrompido por segurança.")
            return "DONE"

        print(f"Alinhando tronco | trunk_aligned={world.trunk_aligned}")
        return "DONE" if world.trunk_aligned else "RUNNING"


class KickAction(ActionElement):
    """Executa o chute. Não deve ser interrompido."""

    def __init__(self):
        super().__init__("KickAction")

    def perform(self, world: WorldState) -> str:
        print(f"Chutando a bola... | kick_done={world.kick_done}")

        if world.kick_done:
            return "DONE"

        return "RUNNING"

    def do_not_reevaluate(self) -> bool:
        return True


class DoneAction(ActionElement):
    """Ação terminal — missão concluída."""

    def __init__(self):
        super().__init__("DoneAction")

    def perform(self, world: WorldState) -> str:
        print("Missão concluída.")
        return "RUNNING"

    def do_not_reevaluate(self) -> bool:
        return True


class SimpleDSD:
    def __init__(self, world: WorldState):
        self.world = world
        self.stack: List[StackElement] = []
        self.last_results: Dict[int, str] = {}

        self.root_decision = RootDecision()
        self.ball_found_decision = BallFoundDecision()
        self.ball_close_decision = BallCloseDecision()
        self.ball_position_decision = BallPositionDecision()
        self.head_kick_check_decision = HeadKickCheckDecision()
        self.hor_motor_decision = HorMotorDecision()
        self.trunk_aligned_decision = TrunkAlignedDecision()

        self.stand_action = StandUpAction()
        self.recovery_action = RecoveryAction()
        self.search_action = SearchBallAction()
        self.go_action = GoToBallAction()
        self.align_head_action = AlignHeadForKickAction()
        self.align_trunk_action = AlignTrunkAction()
        self.kick_action = KickAction()
        self.done_action = DoneAction()

        self.stack.append(self.root_decision)
        self.expand_top_decisions()

    def print_stack(self):
        print("Pilha:", " -> ".join(e.name for e in self.stack))

    def push(self, elem: StackElement):
        self.stack.append(elem)

    def pop(self) -> Optional[StackElement]:
        return self.stack.pop() if self.stack else None

    def map_decision(self, decision: DecisionElement, result: str) -> StackElement:
        if isinstance(decision, RootDecision):
            if result == "RECOVERY":
                return self.recovery_action
            return self.ball_found_decision

        if isinstance(decision, BallFoundDecision):
            return self.search_action if result == "NO" else self.ball_close_decision

        if isinstance(decision, BallCloseDecision):
            return self.trunk_aligned_decision if result == "YES" else self.go_action

        if isinstance(decision, TrunkAlignedDecision):
            return self.head_kick_check_decision if result == "YES" else self.align_trunk_action

        if isinstance(decision, HeadKickCheckDecision):
            return self.kick_action if result == "YES" else self.align_head_action

        if isinstance(decision, BallPositionDecision):
            return self.go_action

        if isinstance(decision, HorMotorDecision):
            return self.search_action

        raise ValueError(f"Mapeamento inválido para: {decision.name} com resultado '{result}'")

    def expand_top_decisions(self):
        while self.stack and isinstance(self.stack[-1], DecisionElement):
            decision = self.stack[-1]
            result = decision.perform(self.world)
            self.last_results[id(decision)] = result
            self.push(self.map_decision(decision, result))

    def reevaluate(self):
        if self.stack and isinstance(self.stack[-1], ActionElement):
            if self.stack[-1].do_not_reevaluate():
                return

        for i, elem in enumerate(self.stack):
            if isinstance(elem, DecisionElement):
                new_result = elem.perform(self.world)
                old_result = self.last_results.get(id(elem))

                if new_result != old_result:
                    while len(self.stack) > i + 1:
                        self.pop()
                    self.last_results[id(elem)] = new_result
                    self.push(self.map_decision(elem, new_result))
                    self.expand_top_decisions()
                    return

    def step(self):
        self.reevaluate()
        if not self.stack:
            return

        top = self.stack[-1]
        self.print_stack()

        if isinstance(top, ActionElement):
            result = top.perform(self.world)
            if result == "DONE":
                self.pop()
                self.expand_top_decisions()


if __name__ == "__main__":
    print("SIMULAÇÃO DSD — ROBÔ JOGADOR")

    world = WorldState()
    dsd = SimpleDSD(world)

    print("\n--- Passo 1: Robô caiu ---")
    world.fall_state = FallState.FRONT
    dsd.step()

    print("\n--- Passo 2: Queda cessou ---")
    world.fall_state = FallState.UP
    dsd.step()

    print("\n--- Passo 3: Procurando bola ---")
    world.ball_found = False
    world.hor_motor_out_of_center = HeadHorizontalPosition.LEFT
    dsd.step()

    print("\n--- Passo 4: Bola encontrada ---")
    world.ball_found = True
    world.ball_position = BallPosition.LEFT_TOP
    world.ball_angle = 0.4
    world.ball_close = False
    dsd.step()

    print("\n--- Passo 5: Bola perto ---")
    world.ball_close = True
    world.ball_position = BallPosition.CENTER
    world.trunk_aligned = False
    world.head_kick_check = False
    dsd.step()

    print("\n--- Passo 6: Tronco alinhado ---")
    world.trunk_aligned = True
    dsd.step()

    print("\n--- Passo 7: Cabeça alinhada ---")
    world.head_kick_check = True
    dsd.step()

    print("\n--- Passo 8: Chute ---")
    world.kick_done = False
    dsd.step()

    print("\n--- Passo 9: Finalizado ---")
    world.kick_done = True
    dsd.step()
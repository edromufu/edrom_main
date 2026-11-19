#!/usr/bin/env python3
"""Compara comando (/goal_joint_states) x feedback (/feedback) junta a junta, a partir dos CSVs
gerados por bag_to_csv.py, considerando os offsets configurados:

  - calibration_offset / inverted por motor, de motors_direct.yaml (edrom_lowlevel)
  - backlash_offset_hp do walking_engine_node (aplicado a r_hip_pitch/l_hip_pitch antes de
    publicar /goal_joint_states)

O binario do direct_controller usado para gravar o bag tinha um bug de ordem entre inversao e
offset na leitura (invertia e so depois subtraia o offset, enquanto a escrita soma o offset
antes de inverter). Para motores invertidos com calibration_offset != 0
(l_hip_roll, r_hip_roll, l_hip_pitch) isso dobra o offset no /feedback:
    feedback_bruto = comando - 2*offset   (em vez de == comando, com rastreamento perfeito)
Esse script reconstroi o valor como seria com a leitura corrigida:
    feedback_corrigido = feedback_bruto + 2*offset  (so para motores invertidos com offset != 0)
Use --no-legacy-fix se o bag foi gravado DEPOIS do fix em direct_controller.cpp (o feedback ja
vem correto e essa correcao nao deve ser aplicada de novo).

Uso:
    python3 plot_feedback_comparison.py [dir_csv] [dir_saida] [--no-legacy-fix]

Default: dir_csv = bags/csv_out, dir_saida = dir_csv/plots
"""
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
MOTORS_YAML = REPO_ROOT / "src/edrom_lowlevel/config/motors_direct.yaml"
BACKLASH_OFFSET_HP = -0.32  # default declarado em walking_engine_node.cpp
BACKLASH_JOINTS = {"r_hip_pitch", "l_hip_pitch"}


def load_motor_config(yaml_path: Path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    params = data["direct_controller"]["ros__parameters"]
    joint_names = params["joint_names"]
    config = {}
    for name in joint_names:
        entry = params.get(name, {})
        config[name] = {
            "inverted": bool(entry.get("inverted", False)),
            "calibration_offset": float(entry.get("calibration_offset", 0.0)),
        }
    return config


def load(csv_dir: Path):
    goal = pd.read_csv(csv_dir / "goal_joint_states.csv")
    fb = pd.read_csv(csv_dir / "feedback.csv")

    t0 = min(goal["stamp_ns"].min(), fb["stamp_ns"].min())
    goal["t"] = (goal["stamp_ns"] - t0) / 1e9
    fb["t"] = (fb["stamp_ns"] - t0) / 1e9
    return goal, fb


def common_joints(goal: pd.DataFrame, fb: pd.DataFrame):
    ignore = {"stamp_ns", "t"}
    goal_joints = {c for c in goal.columns if c not in ignore and goal[c].notna().any()}
    fb_joints = {c for c in fb.columns if c not in ignore and fb[c].notna().any()}
    return sorted(goal_joints & fb_joints)


def apply_legacy_readback_fix(fb: pd.DataFrame, joints, motor_config):
    """Reconstroi o feedback como seria com a leitura corrigida (offset subtraido antes de
    inverter). So afeta motores invertidos com calibration_offset != 0."""
    fixed = fb.copy()
    corrected_joints = []
    for joint in joints:
        cfg = motor_config.get(joint)
        if cfg is None or not cfg["inverted"] or cfg["calibration_offset"] == 0.0:
            continue
        fixed[joint] = fixed[joint] + 2 * cfg["calibration_offset"]
        corrected_joints.append(joint)
    return fixed, corrected_joints


def plot_joint(ax, goal, fb, joint, backlash_note=False):
    ax.plot(goal["t"].to_numpy(), goal[joint].to_numpy(), label="comando", linewidth=1.2, alpha=0.8)
    ax.plot(fb["t"].to_numpy(), fb[joint].to_numpy(), label="feedback", linewidth=1.2, alpha=0.8)
    if backlash_note:
        sem_backlash = goal[joint].to_numpy() - BACKLASH_OFFSET_HP
        ax.plot(goal["t"].to_numpy(), sem_backlash, label="comando sem backlash (IK)",
                 linewidth=1.0, alpha=0.6, linestyle="--")
    ax.set_title(joint, fontsize=9)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.3)


def summarize(goal, fb, joint):
    merged = pd.merge_asof(
        fb[["t", joint]].sort_values("t").rename(columns={joint: "feedback"}),
        goal[["t", joint]].sort_values("t").rename(columns={joint: "comando"}),
        on="t",
        direction="nearest",
        tolerance=0.1,
    ).dropna()
    if merged.empty:
        return None
    erro = (merged["feedback"] - merged["comando"]).abs()
    return {
        "junta": joint,
        "n_amostras": len(merged),
        "erro_medio_rad": erro.mean(),
        "erro_max_rad": erro.max(),
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    apply_fix = "--no-legacy-fix" not in sys.argv

    csv_dir = Path(args[0]) if len(args) > 0 else Path("bags/csv_out")
    out_dir = Path(args[1]) if len(args) > 1 else csv_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    motor_config = load_motor_config(MOTORS_YAML)
    goal, fb = load(csv_dir)
    joints = common_joints(goal, fb)

    if not joints:
        print("Nenhuma junta em comum entre comando e feedback.")
        return

    corrected_joints = []
    if apply_fix:
        fb, corrected_joints = apply_legacy_readback_fix(fb, joints, motor_config)
        if corrected_joints:
            print(f"Corrigindo bug de offset duplicado (bug antigo de leitura) em: {', '.join(corrected_joints)}")
    else:
        print("--no-legacy-fix: usando feedback como gravado, sem correcao de offset.")

    print(f"Juntas em comum ({len(joints)}): {', '.join(joints)}")

    n = len(joints)
    ncols = 3
    nrows = -(-n // ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3 * nrows), squeeze=False)

    rows = []
    for i, joint in enumerate(joints):
        ax = axes[i // ncols][i % ncols]
        backlash_note = joint in BACKLASH_JOINTS
        plot_joint(ax, goal, fb, joint, backlash_note=backlash_note)
        if i == 0:
            ax.legend(fontsize=7)
        stats = summarize(goal, fb, joint)
        if stats:
            stats["offset_corrigido"] = joint in corrected_joints
            rows.append(stats)

    for i in range(n, nrows * ncols):
        axes[i // ncols][i % ncols].axis("off")

    fig.suptitle("Comando vs Feedback por junta (offsets de motors_direct.yaml + backlash do walking_engine)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    grid_path = out_dir / "comparacao_todas_juntas.png"
    fig.savefig(grid_path, dpi=150)
    print(f"Grafico geral salvo em {grid_path}")

    for joint in joints:
        fig2, ax2 = plt.subplots(figsize=(9, 4))
        plot_joint(ax2, goal, fb, joint, backlash_note=joint in BACKLASH_JOINTS)
        ax2.legend()
        fig2.tight_layout()
        p = out_dir / f"{joint}.png"
        fig2.savefig(p, dpi=150)
        plt.close(fig2)

    if rows:
        summary_df = pd.DataFrame(rows).sort_values("erro_medio_rad", ascending=False)
        summary_path = out_dir / "resumo_erros.csv"
        summary_df.to_csv(summary_path, index=False)
        print(f"\nResumo de erro (comando vs feedback mais proximo no tempo, tol=100ms):")
        print(summary_df.to_string(index=False))
        print(f"\nResumo salvo em {summary_path}")

    if plt.get_backend().lower() != "agg":
        plt.show()


if __name__ == "__main__":
    main()

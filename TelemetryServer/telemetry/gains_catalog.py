"""Shared Gains panel catalog: order, legends, tooltips (web + Qt)."""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple, TypedDict

# Sliders / buttons / aliases — not shown as gain fields.
HIDDEN_FROM_GAINS = frozenset(
    {
        "vel_ref_turns_s",
        "heading_ref_rad",
        "pos_reset",
        "heading_reset",
        "heading_inc",
        "heading_dec",
    }
)

TOOLTIPS: Dict[str, str] = {
    "meca_k_grav": (
        "Terme de gravité de u_meca : −k · sin(pitch). Réglé pour que le robot "
        "tienne sans le P quand il est déjà à l’angle d’équilibre. Trop haut = "
        "il pousse trop vers l’avant/l’arrière."
    ),
    "err_k_pitch": (
        "Gain P d’équilibre sur l’erreur d’angle (pitch_ref_effectif − pitch). "
        "C’est le « ressort » qui ramène le robot vertical. Plus haut = plus "
        "raide, plus nerveux."
    ),
    "meca_k_pitch_damp": (
        "Amortisseur visqueux de u_meca : −Kd · θ̇ (N·m·s/rad, pas I θ̈). "
        "Amortit le pompage. Plus haut = plus de frein en rotation, trop haut = "
        "le robot devient mou / vibre."
    ),
    "err_k_vel": (
        "Kv live (id 48, snapshot v13) : u_v = Kv · (v_ref − v_roue), clamp "
        "±0,010 Nm compile-time. Terme d’erreur, pas le FF gravité. Négatif = "
        "freine dans la convention actuelle. Défaut −0,0005. 0 = inactif."
    ),
    "balance_output_alpha": (
        "Filtre du couple de sortie : u = α·u_précédent + (1−α)·u_brut. Plus "
        "proche de 1 = couple plus lisse et plus lent. Coupe le chatter, "
        "ajoute du retard."
    ),
    "pitch_ref_rad": (
        "Biais d’angle ajouté à la consigne de pitch de la cascade (trim). "
        "Sert à compenser un déséquilibre statique (batterie, câbles), pas à "
        "commander la vitesse."
    ),
    "pitch_failsafe_rad": (
        "Angle au-delà duquel on considère la chute (et on clamp le sin du FF). "
        "Défaut ~0,79 rad (45°). Plus bas = coupe plus tôt."
    ),
    "cmd_max_torque_nm": (
        "Saturation du couple commandé à chaque roue après ajout du yaw. "
        "Plafond de sécurité / d’agressivité. Défaut actuel ~0,08 Nm."
    ),
    "friction_mode": (
        "Choix du modèle de frottement ajouté au couple. 0 = deadband legacy "
        "(un seul palier, signé comme u). 1 = deux paliers : statique à "
        "l’arrêt, cinétique en mouvement. Refusé hors de [0, 1]."
    ),
    "friction_static_nm": (
        "En mode 1, couple de décollage quand |ω| ≤ ε. Signé comme le couple "
        "commandé (u). Trop haut = à-coup au démarrage. Refusé si < 0."
    ),
    "friction_kinetic_nm": (
        "En mode 1, couple Coulomb une fois en mouvement (|ω| > ε). Signé "
        "comme la vitesse de roue. Refusé si < 0."
    ),
    "friction_vel_eps_turns_s": (
        "Seuil |ω| qui sépare statique et cinétique en mode 1. En dessous = "
        "breakaway, au-dessus = Coulomb. Refusé si ≤ 0."
    ),
    "torque_deadband_nm": (
        "En mode 0 seulement : ajoute sign(u)·D au couple (deadband / Coulomb "
        "unique). Ignoré si friction_mode = 1."
    ),
    "torque_deadband_pitch_max_rad": (
        "Coupe toute compensation de frottement si |pitch| dépasse ce seuil "
        "(évite de pousser quand on est déjà penché). Si 0, la friction est "
        "coupée dès que pitch ≠ 0."
    ),
    "torque_deadband_rate_max_rads": (
        "Coupe la compensation de frottement si |vitesse de tangage| dépasse "
        "ce seuil. Évite d’ajouter du couple pendant un mouvement rapide."
    ),
    "motor_torque_correction_kp": (
        "Gain P de la rustine moteur : dτ = Kp · (α_ref − α_meas), avec "
        "α_ref = (u − c·sign(ω)) / J et α_meas = dω/dt encodeur. 0 = bloc "
        "entièrement inactif. Ce n’est pas un gain d’équilibre pitch."
    ),
    "motor_torque_correction_max_nm": (
        "Plafond de la rustine dτ par roue, pour qu’une mauvaise estimée de J "
        "ou un bruit d’α_meas ne fasse pas un gros à-coup."
    ),
    "motor_J": (
        "Inertie d’arbre moteur utilisée pour calculer l’accélération attendue "
        "α_ref = (u − c·sign(ω)) / J. N’a d’effet que si "
        "motor_torque_correction_kp > 0."
    ),
    "motor_friction_c": (
        "Frottement soustrait à u avant de diviser par J. N’a d’effet que si "
        "motor_torque_correction_kp > 0."
    ),
    "motor_torque_correction_gate_pitch_max_rad": (
        "Si |pitch| dépasse ce seuil, dτ moteur = 0 (rustine seulement près "
        "de la verticale). Ignoré si kp = 0."
    ),
    "motor_torque_correction_gate_rate_max_rads": (
        "Si |θ̇| dépasse ce seuil, dτ moteur = 0. Ignoré si kp = 0."
    ),
    "motor_torque_correction_gate_vel_max_turns_s": (
        "Si |ω moteur| dépasse ce seuil, dτ = 0. 0 = pas de gate vitesse. "
        "Ignoré si kp = 0."
    ),
    "motor_accel_lpf": (
        "Lissage de α_meas (dω/dt encodeur). Plus proche de 1 = mesure plus "
        "calme, plus de retard. N’a d’effet que si kp > 0."
    ),
    "cascade_vel_kp": (
        "Gain P de la cascade vitesse. Multiplie l’erreur (v_ref − v_roue) "
        "pour demander une inclinaison. Plus haut = le robot se penche plus "
        "fort pour rattraper la vitesse."
    ),
    "cascade_vel_kd": (
        "Gain D sur l’accélération de roue mesurée (v̇ filtré), pas sur la "
        "dérivée de l’erreur. Freine les oscillations de la cascade quand la "
        "vitesse change vite."
    ),
    "cascade_vel_err_ema_alpha": (
        "Mémoire de l’intégrateur sur l’erreur de vitesse : e_f = α·e_f + "
        "(1−α)·e. Plus proche de 1 = l’erreur passée compte plus longtemps. "
        "0 = pas de mémoire. Refusé hors de [0, 1)."
    ),
    "cascade_vel_ema_kp": (
        "Gain sur l’erreur de vitesse mémorisée (e_f). Rattrape un biais "
        "persistant que le Kp seul laisse passer. 0 = intégrateur inactif."
    ),
    "cascade_vel_accel_kp": (
        "Inclinaison anticipée quand on accélère la consigne (v̇_ref). "
        "Compense le lean nécessaire pour accélérer, sans attendre l’erreur "
        "de vitesse."
    ),
    "vel_ref_slew_turns_s2": (
        "Limite la vitesse à laquelle v_ref peut changer. 0 = pas de limite. "
        "Défaut 80 ≈ 0,5 s pour aller à ~2 m/s."
    ),
    "cascade_pitch_ref_max_rad": (
        "Plafond d’inclinaison demandée par la cascade (et du pitch_ref "
        "effectif). Défaut ~0,26 rad (15°)."
    ),
    "heading_kp": (
        "Gain P sur l’erreur de vitesse de lacet (ψ̇_ref − ψ̇). Produit un "
        "couple différentiel. 0 (avec kd = 0) coupe toute la boucle yaw."
    ),
    "heading_kd": (
        "Gain D sur ψ̈_f : dérivée du gyro brut, puis EMA (heading_d_ema). "
        "Amortit les à-coups de lacet. 0 = D off. Refusé si < 0."
    ),
    "heading_torque_max_nm": (
        "Plafond du couple différentiel (un côté +, l’autre −). 0 désactive "
        "le yaw. Refusé si < 0."
    ),
    "heading_ema": (
        "EMA sur ψ̇ gyro avant le P : y = α·y + (1−α)·ψ̇_brut. Plus proche de "
        "1 = plus de retard. 0 = gyro brut. Défaut 0,90 ≈ 20 ms à 500 Hz. "
        "Live id 49. Refusé hors de [0, 1)."
    ),
    "heading_d_ema": (
        "EMA après dψ̇/dt du gyro brut, pour le D. 0,96 ≈ 50 ms à 500 Hz. "
        "0 = ψ̈ brut. Live id 50, snapshot v15. Refusé hors de [0, 1)."
    ),
    "antipat_enable": (
        "Active l’antipatinage (FSM lift / both-air / recovery). 0 = off "
        "(défaut, pas de couple overlay). > 0,5 = on. Refusé hors de [0, 1]."
    ),
    "antipat_sync_enable": (
        "Autorise le sync unilatéral (une roue en l’air). 0 = off (défaut). "
        "Indépendant de both-air. Refusé hors de [0, 1]."
    ),
    "antipat_both_enable": (
        "Autorise BOTH_AIR (deux roues). 0 = off (défaut) : seule la voie "
        "une roue (SYNC) reste possible si sync_enable = 1. Refusé hors de [0, 1]."
    ),
    "antipat_sync_track_width_m": (
        "Voie (m) pour ψ̇_cin = (v_r − v_l) / L. Trop petit = e_ψ explosif. "
        "Refusé si ≤ 0."
    ),
    "antipat_tau_min_nm": (
        "Plancher |τ| au dénominateur de η = |α| / max(|τ|_ema, τ_min). Évite "
        "η infini à couple nul. Refusé si ≤ 0."
    ),
    "antipat_tau_ema": (
        "EMA sur |τ| réellement envoyé, uniquement pour η "
        "(pas la consigne moteur) : y = α·y + (1−α)·|cmd|. "
        "Évite qu’un tick à τ≈0 fasse exploser η. 0 = |τ| brut. "
        "Défaut 0,85 ≈ 6 ms à 1 kHz. Live id 77, snapshot v17. "
        "Refusé hors de [0, 1)."
    ),
    "antipat_eta_on": (
        "Seuil η pour candidater lift / both-air. Plus haut = moins sensible."
    ),
    "antipat_both_eta_off": (
        "BOTH_AIR only: both η must fall under this to count as recontact. "
        "Must stay below antipat_eta_on. Unused by SYNC."
    ),
    "antipat_both_alpha_contact_max_rads2": (
        "Sortie both-air alternative : |α| sous ce seuil. 0 = ignoré "
        "(sortie sur η seulement)."
    ),
    "antipat_both_tau_steady_air_nm": (
        "Si |τ_both| reste sous ce plancher et |ω| est encore haut, on "
        "considère que ça vole encore (pas de recontact)."
    ),
    "antipat_omega_air_min_turns_s": (
        "Shared: still-flying latch. SYNC: |ω_air − ω_sol| above this blocks "
        "exit. BOTH_AIR: |ω| above this with tiny τ_both blocks recontact."
    ),
    "antipat_sync_k_dom": (
        "Une roue n’est candidate lift que si η_cette > k_dom · max(η_autre, 1). "
        "Plus haut = il faut une domination plus nette."
    ),
    "antipat_sync_eps_abs_rads": (
        "Seuil absolu |ψ̇ − ψ̇_cin| pour déclarer décorrélation yaw/roues."
    ),
    "antipat_sync_k_rel": (
        "Seuil relatif : décorrélé si e_ψ > k_rel · |ψ̇|. Le max(abs, rel) "
        "est utilisé."
    ),
    "antipat_sync_k_off": (
        "SYNC exit hysteresis: recorrelated if e_ψ < k_off · "
        "max(eps_abs, k_rel·|ψ̇|) for t_off. η_off is not used."
    ),
    "antipat_sync_t_on_ms": (
        "Temps (ms) que la candidate unilatérale doit tenir avant SYNC_L/R."
    ),
    "antipat_sync_t_off_ms": (
        "Temps (ms) de condition de sortie avant recovery (unilatéral)."
    ),
    "antipat_both_t_on_ms": (
        "Temps (ms) que both-cand doit tenir avant BOTH_AIR."
    ),
    "antipat_both_t_off_ms": (
        "Temps (ms) de condition de recontact avant recovery (both-air)."
    ),
    "antipat_both_t_ma_ms": (
        "Fenêtre (ms) de moyenne pour v_good (vitesse « saine » avant lift)."
    ),
    "antipat_t_recover_ms": (
        "Durée (ms) du ramp de recovery avant retour NORMAL."
    ),
    "antipat_both_u_min_nm": (
        "Plancher |u| pour le test pitch_mismatch (both-air). Sous ce couple "
        "on n’entre pas en both-air."
    ),
    "antipat_both_pitch_rate_min_rads": (
        "Plancher |θ̇| pour pitch_mismatch. Évite un both-air sur du bruit."
    ),
    "antipat_sync_k": (
        "SYNC P gain: τ_sync = antipat_sync_k · (ω_sol − ω_air) + "
        "antipat_sync_kd · ė. Unit N·m / (turn/s)."
    ),
    "antipat_sync_kd": (
        "SYNC D gain on ė from alpha_*_rads2 / 2π (EMA motor_accel_lpf). "
        "0 = P only. Live id 79, snapshot v19. Refused if < 0."
    ),
    "antipat_sync_tau_max_nm": (
        "Plafond du couple de sync unilatéral par roue. Refusé si < 0."
    ),
    "antipat_u_fade_ms": (
        "Shared linear ramp of u on a lifted wheel (SYNC or BOTH_AIR): "
        "1→0 on lift, 0→1 on recovery. 0 = step. Live id 78. Refused if < 0."
    ),
    "antipat_both_k_v": (
        "Gain both-air : τ = k · (v_good − ω) pour chaque roue."
    ),
    "antipat_both_tau_max_nm": (
        "Plafond du couple both-air par roue. Refusé si < 0."
    ),
    "pos_kp": (
        "Gain P de position : transforme l’erreur x_ref − x en consigne de "
        "vitesse. 0 = la position ne commande plus v_ref (sauf via pos_ema_kp)."
    ),
    "pos_kd": (
        "Amortissement sur la vitesse mesurée, soustrait à v_ref. Évite que "
        "la boucle position pousse trop fort quand on roule déjà vers la cible."
    ),
    "pos_v_max_turns_s": (
        "Plafond de la consigne de vitesse issue de la boucle position. "
        "Refusé si < 0."
    ),
    "pos_err_ema_alpha": (
        "Mémoire de l’erreur de position e_f = α·e_f + (1−α)·x_err. Plus "
        "proche de 1 = l’intégrateur oublie plus lentement."
    ),
    "pos_ema_kp": (
        "Gain sur l’erreur de position mémorisée. Rattrape un offset que "
        "pos_kp laisse. 0 = pas d’intégrateur."
    ),
    "wheel_radius_m": (
        "Rayon de roue pour passer des tours moteur à des mètres. Faux rayon "
        "= fausse échelle. Refusé si ≤ 0. Défaut 0,04 m."
    ),
    "outer_mode": (
        "0 = la cascade suit vel_ref. 1 = une boucle position calcule vel_ref "
        "à partir de x_ref. Les deux modes sont exclusifs. Passer en 1 fait "
        "un pos_reset."
    ),
    "pos_x_ref_m": (
        "Position cible en mètres, utilisée seulement si outer_mode = 1."
    ),
    "strategy": (
        "Sélecteur de stratégie de contrôle. 0 = ff_cascade (seule stratégie "
        "utilisée aujourd’hui). Changer de valeur réinitialise la stratégie "
        "active."
    ),
    "wheel_encoder_vel_lpf_alpha": (
        "Filtre de la vitesse de roue mesurée (la v utilisée par la cascade "
        "et l’affichage). Plus proche de 1 = plus lisse, plus de retard sur v."
    ),
}


class PanelSection(TypedDict):
    title: str
    legend: str
    fields: Sequence[str]


class PanelSpec(TypedDict, total=False):
    title: str
    legend: str
    fields: Sequence[str]
    sections: Sequence[PanelSection]


def panel_field_names(panel: PanelSpec) -> Tuple[str, ...]:
    names = list(panel.get("fields") or ())
    for section in panel.get("sections") or ():
        names.extend(section["fields"])
    return tuple(names)


# Order is the screen order: balance, plant compensation, vel, heading,
# antipatinage, position, system.
PANELS: Sequence[PanelSpec] = (
    {
        "title": "Équilibre",
        "legend": (
            "Pitch → couple : u_meca = −k·sin(θ) − Kd·θ̇ ; "
            "u_err = Kp·θ_err + Kv·(v_ref − v) ; puis LPF."
        ),
        "fields": (
            "meca_k_grav",
            "err_k_pitch",
            "meca_k_pitch_damp",
            "err_k_vel",
            "balance_output_alpha",
            "pitch_ref_rad",
            "pitch_failsafe_rad",
            "cmd_max_torque_nm",
        ),
    },
    {
        "title": "Friction / deadband / motor correction",
        "legend": (
            "Couple hors pitch : u += friction(ω, u) ; "
            "dτ = Kp·(α_ref − α_meas), α_ref = (u − c·sign(ω))/J. "
            "Kp = 0 → rustine off."
        ),
        "fields": (
            "friction_mode",
            "friction_static_nm",
            "friction_kinetic_nm",
            "friction_vel_eps_turns_s",
            "torque_deadband_nm",
            "torque_deadband_pitch_max_rad",
            "torque_deadband_rate_max_rads",
            "motor_torque_correction_kp",
            "motor_torque_correction_max_nm",
            "motor_J",
            "motor_friction_c",
            "motor_torque_correction_gate_pitch_max_rad",
            "motor_torque_correction_gate_rate_max_rads",
            "motor_torque_correction_gate_vel_max_turns_s",
            "motor_accel_lpf",
        ),
    },
    {
        "title": "Vitesse",
        "legend": (
            "v → pitch : pitch_cmd = Kp·e + Kema·e_f + Kd·v̇ − Kacc·v̇_ref, "
            "puis pitch_trim = −pitch_cmd."
        ),
        "fields": (
            "cascade_vel_kp",
            "cascade_vel_kd",
            "cascade_vel_err_ema_alpha",
            "cascade_vel_ema_kp",
            "cascade_vel_accel_kp",
            "vel_ref_slew_turns_s2",
            "cascade_pitch_ref_max_rad",
        ),
    },
    {
        "title": "Heading",
        "legend": (
            "Lacet → couple différentiel : ψ̇_p = EMA(gyro, heading_ema) ; "
            "ψ̈_f = EMA(d gyro/dt, heading_d_ema) ; "
            "u_yaw = Kp·(ψ̇_ref − ψ̇_p) − Kd·ψ̈_f."
        ),
        "fields": (
            "heading_kp",
            "heading_kd",
            "heading_torque_max_nm",
            "heading_ema",
            "heading_d_ema",
        ),
    },
    {
        "title": "Antipatinage",
        "legend": (
            "Shared: η = |alpha_rads2| / max(EMA(|cmd|), tau_min). "
            "enable = 0 → FSM off. Sync/Both panels are exclusive to that path."
        ),
        "fields": (
            "antipat_enable",
            "antipat_tau_min_nm",
            "antipat_tau_ema",
            "antipat_eta_on",
            "antipat_omega_air_min_turns_s",
            "antipat_t_recover_ms",
            "antipat_u_fade_ms",
        ),
        "sections": (
            {
                "title": "Sync",
                "legend": (
                    "One wheel: τ_sync = antipat_sync_k·(ω_sol − ω_air) + "
                    "antipat_sync_kd·ė. Exit: recorrelated for t_off, blocked "
                    "while |ω_air − ω_sol| > omega_air_min."
                ),
                "fields": (
                    "antipat_sync_enable",
                    "antipat_sync_track_width_m",
                    "antipat_sync_k_dom",
                    "antipat_sync_eps_abs_rads",
                    "antipat_sync_k_rel",
                    "antipat_sync_k_off",
                    "antipat_sync_t_on_ms",
                    "antipat_sync_t_off_ms",
                    "antipat_sync_k",
                    "antipat_sync_kd",
                    "antipat_sync_tau_max_nm",
                ),
            },
            {
                "title": "Both",
                "legend": (
                    "Two wheels: cut u, P toward v_good. "
                    "antipat_both_enable = 0 → this path is dead."
                ),
                "fields": (
                    "antipat_both_enable",
                    "antipat_both_eta_off",
                    "antipat_both_t_on_ms",
                    "antipat_both_t_off_ms",
                    "antipat_both_t_ma_ms",
                    "antipat_both_u_min_nm",
                    "antipat_both_pitch_rate_min_rads",
                    "antipat_both_k_v",
                    "antipat_both_tau_max_nm",
                    "antipat_both_alpha_contact_max_rads2",
                    "antipat_both_tau_steady_air_nm",
                ),
            },
        ),
    },
    {
        "title": "Position",
        "legend": (
            "x → v_ref (si outer_mode = 1) : "
            "v_ref = clamp(Kp·x_err + Kema·e_f − Kd·v, ±v_max), "
            "puis cascade vitesse."
        ),
        "fields": (
            "outer_mode",
            "pos_x_ref_m",
            "pos_kp",
            "pos_kd",
            "pos_v_max_turns_s",
            "pos_err_ema_alpha",
            "pos_ema_kp",
            "wheel_radius_m",
        ),
    },
    {
        "title": "Système",
        "legend": (
            "Mesure et loi : v_roue = LPF(encodeur) ; "
            "strategy sélectionne la loi de commande."
        ),
        "fields": (
            "strategy",
            "wheel_encoder_vel_lpf_alpha",
        ),
    },
)

CATEGORIES: Sequence[Tuple[str, Sequence[str]]] = tuple(
    (panel["title"], panel_field_names(panel)) for panel in PANELS
)


class WireField(TypedDict):
    name: str
    tooltip: str


class WireSection(TypedDict):
    title: str
    legend: str
    fields: List[WireField]


class WirePanel(TypedDict, total=False):
    title: str
    legend: str
    fields: List[WireField]
    sections: List[WireSection]


def _wire_fields(names: Sequence[str]) -> List[WireField]:
    return [{"name": name, "tooltip": TOOLTIPS.get(name, "")} for name in names]


def panels_for_wire() -> List[WirePanel]:
    """JSON-ready catalog for get_params (web Gains)."""
    out: List[WirePanel] = []
    for panel in PANELS:
        item: WirePanel = {
            "title": panel["title"],
            "legend": panel["legend"],
            "fields": _wire_fields(panel.get("fields") or ()),
        }
        sections = panel.get("sections") or ()
        if sections:
            item["sections"] = [
                {
                    "title": section["title"],
                    "legend": section["legend"],
                    "fields": _wire_fields(section["fields"]),
                }
                for section in sections
            ]
        out.append(item)
    return out

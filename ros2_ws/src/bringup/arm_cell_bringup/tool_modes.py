"""Shared grasp mode vocabulary for Tool profiles, configuration export and the launcher."""

GRASP_MODE_LABELS = ('논리 판정 + 고정', '물리 판정 + 고정', '물리 파지')


def tool_mode_choices(tool):
    """Return one classified mode per supported pair, in the Tool's selection order."""
    runtime = tool['runtime']
    choices = []
    for pair in runtime.get('supported_combinations', [['logical', 'fixed_attach']]):
        modes = [mode for mode in runtime.get('launcher_modes', []) if mode.get('combination') == pair]
        if len(modes) != 1 or modes[0].get('label') not in GRASP_MODE_LABELS:
            raise ValueError(f'{pair} 조합의 파지 모드에 공통 이름 하나를 지정하세요: {", ".join(GRASP_MODE_LABELS)}')
        choices.append(modes[0])
    return choices

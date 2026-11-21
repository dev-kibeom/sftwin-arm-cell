"""Identity of a Conveyor test and its persistent command receipts."""
from pathlib import Path


def decode_epoch(words):
    capability, epoch = words
    if capability == 0 and epoch == 0:
        return None  # Original application, before Initialize support.
    if capability not in (0x1A17, 0x1A18, 0x1A19):
        raise ValueError('Unknown Conveyor Initialize capability')
    return epoch


def receipt_path(base, epoch):
    path = Path(base).expanduser()
    return str(path.with_name(f'{path.stem}.test-{epoch}{path.suffix}')) if epoch else str(base)


def reserve_test_stream(base, current_epoch, pending):
    """Reserve one fresh receipt namespace for this Initialize, including retries."""
    if pending is not None and pending.is_file():
        epoch = int(pending.read_text())
        if epoch < current_epoch or not 1 <= epoch <= 65535:
            raise ValueError('Initialize belongs to an older PLC stream')
        return epoch
    for epoch in range(current_epoch + 1, 65536):
        path = Path(receipt_path(base, epoch))
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            path.touch(exist_ok=False)
        except FileExistsError:
            continue
        if pending is not None:
            temporary = pending.with_suffix('.tmp')
            temporary.write_text(str(epoch))
            temporary.replace(pending)
        return epoch
    raise ValueError('No unused PLC test number remains')

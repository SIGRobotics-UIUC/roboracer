"""Concise terminal messages alongside the preserved lifecycle evidence."""


def say(message):
    print('[benchmark] '+message, flush=True)


def lap_message(index, warmup, requested, duration=None, valid=False):
    phase = (f'Warm-up lap {index}/{warmup}' if index <= warmup else
             f'Measurement lap {index-warmup}/{requested}')
    timing = f'{duration:.3f} s (native simulator)' if valid and duration else 'native lap time unavailable'
    return f'{phase} completed — {timing}.'


def result_message(status, completed, requested, failure, confirmation):
    text = f'Run {status}: {completed}/{requested} measured laps completed.'
    if failure:
        text += f' Reason: {failure}'
    text += f' Stop confirmation: {confirmation or "not performed (motion was not enabled)"}.'
    return text

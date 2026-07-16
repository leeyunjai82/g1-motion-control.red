#!/usr/bin/env python3
"""
TTS 클라이언트.
서버: http://127.0.0.1:59530/v1/tts (isPlay=1로 서버측 재생)

특징:
  - 내부 큐 + 워커 스레드로 직렬 처리
  - 매 멘트 끝나기 전에 다음 호출이 와도 끊기지 않음
    (글자 수 기반 추정 시간만큼 워커가 대기)
  - speak()는 큐에 enqueue 후 즉시 리턴 (비동기)

사용:
    tts = TextToSpeech()
    tts.speak("Hello world")   # 큐에 넣음, 즉시 리턴
"""

import urllib.parse
import urllib.request
import threading
import queue
import time


class TextToSpeech:
    def __init__(self,
                 host: str = "127.0.0.1",
                 port: int = 59530,
                 voice: int = 6,
                 lang: str = "en",
                 timeout: float = 30.0,
                 chars_per_sec: float = 5.0,
                 verbose: bool = True):
        """
        chars_per_sec: 영어 기준 ~5자/초 (재생 시간 추정용, 보수적)
        """
        self.base_url = f"http://{host}:{port}/v1/tts"
        self.voice    = voice
        self.lang     = lang
        self.timeout  = timeout
        self.chars_per_sec = chars_per_sec
        self.verbose  = verbose
        self._last_text = None
        self._last_time = 0.0
        self._queue = queue.Queue()
        threading.Thread(target=self._worker, daemon=True).start()

    def _build_url(self, text: str, voice: int = None, lang: str = None,
                   static: int = 0, is_play: int = 1) -> str:
        params = {
            "text":   text,
            "voice":  voice if voice is not None else self.voice,
            "lang":   lang if lang is not None else self.lang,
            "static": static,
            "isPlay": is_play,
        }
        return self.base_url + "?" + urllib.parse.urlencode(params)

    def _send(self, url: str) -> bool:
        try:
            req = urllib.request.Request(url, headers={"accept": "*/*"})
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                resp.read()
                if self.verbose:
                    print(f"[TTS] HTTP {resp.status}")
                return 200 <= resp.status < 300
        except Exception as e:
            print(f"[TTS] 오류: {e}")
            return False

    def _worker(self):
        """큐에서 하나씩 꺼내 직렬 처리. _send가 재생 끝까지 대기하므로 sleep 거의 없음."""
        while True:
            item = self._queue.get()
            try:
                text, voice, lang = item
                if self.verbose:
                    print(f"[TTS] {text}")
                url = self._build_url(text, voice=voice, lang=lang)
                self._send(url)
                # _send가 재생 끝까지 대기하므로 추가 대기는 호흡용으로만
                time.sleep(0.3)
            finally:
                self._queue.task_done()

    def wait_until_done(self):
        """큐의 모든 멘트가 처리될 때까지 대기."""
        self._queue.join()

    def speak(self, text: str, voice: int = None, lang: str = None,
              dedup_sec: float = 0.5) -> bool:
        """큐에 enqueue 후 즉시 리턴. dedup_sec 내 동일 텍스트는 무시."""
        if not text:
            return False

        now = time.time()
        if (text == self._last_text) and (now - self._last_time < dedup_sec):
            return False
        self._last_text = text
        self._last_time = now

        self._queue.put((text, voice, lang))
        return True


# 모듈 레벨 기본 인스턴스 (간편 사용)
_default_tts = None


def get_default_tts() -> TextToSpeech:
    global _default_tts
    if _default_tts is None:
        _default_tts = TextToSpeech()
    return _default_tts


def speak(text: str, **kwargs):
    return get_default_tts().speak(text, **kwargs)


if __name__ == "__main__":
    # 모든 멘트 테스트
    MESSAGES = [
        # 시퀀스 멘트
        "Hi, Red Hat Summit! I have gifts for you.",
        "A box! Let me pick it up.",
        "I got it.",
        "This is for you! Please grab the top of the box.",
        "Enjoy your gift!",
        "Nobody? I will put it back.",
        "Who is next? Bring me another box.",
        # 잡소리
        "Welcome to Red Hat Summit 2026!",
        "Free Owala tumbler! Come and get one.",
        "Bring me a box, and I will give you a gift.",
        "I run on RHEL.",
        "I run on open source.",
        "I am faster than OpenShift.",
        "I run on Intel Panther Lake.",
        "I have Intel inside.",
        "My software is from Circulus, Korea.",
        "Circulus from Korea made my software.",
    ]

    tts = TextToSpeech()
    print(f"=== {len(MESSAGES)}개 멘트 큐에 enqueue ===")
    for msg in MESSAGES:
        tts.speak(msg)

    print(f"=== 큐 처리 끝까지 대기 ===\n")
    tts.wait_until_done()
    print("\ndone")

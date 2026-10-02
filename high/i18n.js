/* i18n.js — G1 웹 UI 한/영 전환 (launcher :80 / robot_web :50000 / dashboard :50003 / simulator :8000 공용)
 *
 * 동작
 *  - 화면의 한글 텍스트를 아래 사전으로 영어로 바꾼다 (텍스트 노드 + title/placeholder + alert/confirm/prompt).
 *  - JS 가 나중에 그리는 텍스트도 MutationObserver 로 자동 번역.
 *  - 원문은 노드에 보관 → 한국어로 되돌리면 복원.
 *  - 사전에 없는 한글은 그대로 둔다.
 *  - 언어: ?lang=en|ko → localStorage('g1_lang') → 기본 ko
 *    (포트가 다르면 localStorage 가 따로라, iframe 은 ?lang= / postMessage 로 맞춘다)
 *  - 버튼: id="lang-slot" 요소가 있으면 그 안, 없으면 우측 하단 고정. ?embed=1 이면 버튼 숨김.
 */
(function () {
  'use strict';

  // ---------- 사전 (ko → en). 긴 문구가 먼저 적용된다 ----------
  const DICT = {
    // ===== launcher =====
    '웹 버튼은 비상정지가 아닙니다 — E-STOP/리모컨을 손에 두세요': 'Web buttons are NOT an emergency stop — keep the E-STOP / remote in hand',
    '순서로, 로봇이 자리 잡은 걸 보고 다음 단계를 누르세요': 'in order — press the next step once the robot has settled',
    '일어서기:': 'Stand up:',
    '자세 (FSM)': 'Posture (FSM)',
    '현재 FSM': 'Current FSM',
    '힘 빼기 · 항상 가능': 'power off · always allowed',
    '밸런스 없음 · 1 / 501 에서': 'no balance · from 1 / 501',
    '밸런스 · arm_sdk · 4 에서': 'balance · arm_sdk · from 4',
    '밸런스 없음 · 서 있을 때': 'no balance · while standing',
    'FSM 501 에서만 · 6개 서버': 'FSM 501 only · 6 servers',
    'Robot 시작': 'Start Robot',
    'Robot 정지': 'Stop Robot',
    'Ctrl+C 와 동일': 'same as Ctrl+C',
    '로봇 서버': 'Robot servers',
    '지금 서 있습니다 — 힘이 빠져 넘어집니다!': 'It is standing — it will lose power and fall!',
    '현재 상태 확인 불가 — 서 있다면 힘이 빠져 넘어집니다!': 'State unknown — if standing, it will lose power and fall!',
    '모터 힘이 빠집니다': 'Motors will go limp',
    '· 로봇을 사람이 받치고 있거나 스탠드에 묶여 있습니까?': '· Is someone supporting the robot, or is it tied to the stand?',
    '· 주변에 사람/장애물이 없습니까?': '· Is the area clear of people / obstacles?',
    '로봇이 천천히 앉습니다 (밸런스 제어 없음)': 'The robot sits down slowly (no balance control)',
    '· 팔을 몸 옆으로 내렸습니까?': '· Are the arms lowered to the sides?',
    '· 앉는 동안 로봇을 받치고 있습니까?': '· Are you supporting the robot while it sits?',
    '· 엉덩이 아래 공간이 비어 있습니까?': '· Is the space under the hips clear?',
    'start_robot.sh 를 실행합니다.': 'Run start_robot.sh.',
    '· arm_server 가 기동하면서 팔/허리 제어권(weight=1)을 잡습니다.': '· arm_server takes arm/waist control (weight=1) on startup.',
    '로봇 서버를 정지합니다 (팔 제어권 반납 후 종료).': 'Stop the robot servers (release arm control, then exit).',
    '순서 제한 없음': 'no order restriction',
    '마지막 전송': 'last sent',
    '밸런스 제어': 'balance control',
    '밸런스 없음': 'no balance',
    'launcher 연결 끊김': 'launcher disconnected',
    'Robot 서버 실행 중 — 먼저 [Robot 정지]': 'Robot servers running — press [Stop Robot] first',
    '— 1 → 4 → 501 로 Walk(3DoF waist) 진입 후 시작': '— enter Walk (3DoF waist) via 1 → 4 → 501 first',
    '4 는 FSM 1(Damping) 또는 501 에서만': '4 only from FSM 1 (Damping) or 501',
    '501 은 FSM 4(Lock Standing) 에서만': '501 only from FSM 4 (Lock Standing)',
    'Sit 은 서 있을 때만': 'Sit only while standing',
    '대기 취소 — Damp 로 교체': 'pending cancelled — replaced by Damp',
    '전송 직전 거부': 'rejected right before sending',
    '전송 완료': 'sent',
    '대기/전송 중': 'pending / sending',
    'FSM 조회 실패': 'FSM query failed',
    'LocoClient 초기화 실패': 'LocoClient init failed',
    'FSM 확인 불가': 'FSM unknown',
    '501 상태인지 직접 확인할 것': 'check manually that it is in 501',
    '종료 지연 — 프로세스 그룹 SIGKILL': 'stop timed out — SIGKILL process group',
    '알 수 없는 단계': 'unknown step',
    '위치제어': 'position control',

    // ===== robot_web =====
    'Marker 인식': 'Marker detection',
    'Box 인식': 'Box detection',
    '정지·기본자세': 'stop · home pose',
    'Arm 제어권': 'Arm control',
    '잡기/운반': 'grab / carry',
    '상태 확인 중': 'checking state',
    '마커 추종 시작': 'Marker follow started',
    '마커 추종': 'Marker follow',
    '추종 시작을 누르세요': 'press Start follow',
    '추종 시작': 'Start follow',
    '추종 정지': 'Stop follow',
    '추종 불가:': 'Cannot follow:',
    '앞 거리 X': 'Forward X',
    '좌우 Y': 'Lateral Y',
    '팔·허리 점유 — 잡기/운반': 'arm·waist held — grab / carry',
    '보행 제어기가 팔 사용 — 걷기': 'locomotion controls arms — walking',
    '팔/허리 점유 (전환 중...)': 'arm/waist held (switching...)',
    '걷기 모드 (전환 중...)': 'walk mode (switching...)',
    '완료까지 조작 대기': 'wait until done',
    '상태 확인 불가': 'state unknown',
    '응답 없음': 'no response',
    '마커 정면 도착 완료': 'arrived in front of marker',
    '정면 경유점으로 이동': 'to front waypoint',
    '마커로 접근': 'approaching marker',
    '마커 안 보임': 'marker not visible',
    '마커 놓침': 'Marker lost',
    '수동 정지': 'manual stop',
    '허리 정렬 중 — 잠시 후 보행': 'aligning waist — walking shortly',
    '잡는 중': 'grabbing',
    '모션 실행': 'motion running',
    'AI 인식': 'AI vision',
    '처리 속도': 'Throughput',
    '박스 W×D×H': 'Box W×D×H',
    '마지막 잡기': 'Last grab',
    '기본 자세': 'home pose',
    '기본자세': 'home pose',
    // 잡기 단계 (robot_server GRAB_STAGES, 칸은 띄어쓰기 없이 표시)
    '허리 정렬': 'Waist align', '허리정렬': 'Waist',
    '위쪽 접근': 'Approach', '위쪽접근': 'Approach',
    '측면 하강': 'Lower', '측면하강': 'Lower',
    '받기 대기': 'Wait take', '받기대기': 'Wait take',
    '재검출': 'Re-detect', '잡기': 'Grip', '들기': 'Lift', '건네기': 'Hand over',
    '놓기': 'Release', '복귀': 'Return',
    // robot_server 오류
    '동작 중 - 정지 후 전환': 'busy - stop before switching',
    '잡기/모션 동작 중': 'grab / motion in progress',
    '마커 미검출 - 50011/마커 위치 확인': 'marker not detected - check 50011 / marker position',
    '이미 추종 중': 'already following',
    '파일 파싱 오류': 'file parse error',
    '파일 없음': 'file not found',
    '빈 모션': 'empty motion',
    '검출 없음': 'nothing detected',
    'arm 전환 중': 'arm switching',
    '미초기화': 'not initialized',
    '동작 중': 'busy',

    // ===== dashboard =====
    '인식 박스 · 폭': 'Detected box · W',
    '손 목표 (IK)': 'Hand target (IK)',
    '인식 박스': 'Detected box',
    '폭': 'W',
    '비교': 'Compare',
    '차이': 'diff',
    '기울기': 'tilt',
    '평면': 'plane',
    // 추정 방식 비교 (robot_web AI 오버레이)
    '추정 비교 · 사용': 'Estimator · using',
    '윗면 기울기': 'Top tilt',
    '파지점 차이 L/R': 'Grip pt diff L/R',
    '중심 차이': 'Center diff',
    '높이 기본/평면': 'Height legacy/plane',
    '로봇 연결 시 표시': 'shown when the robot is connected',
    '부하 (|토크| / 최대 토크)': 'Load (|torque| / max torque)',
    '모터 온도': 'Motor temp',
    '데이터 없음': 'no data',

    // ===== simulator =====
    'Joint · 관절': 'Joint',
    'IK · 좌표': 'IK · Cartesian',
    '손: 확인 중...': 'Hand: checking...',
    '손: 연결됨': 'Hand: connected',
    '손: 미연결': 'Hand: not connected',
    '홈 자세': 'Home pose',
    '오른쪽 팔': 'Right arm',
    '왼쪽 팔': 'Left arm',
    '허리 (Waist)': 'Waist',
    'X (전후)': 'X (fwd/back)',
    'Y (좌우)': 'Y (left/right)',
    'Z (상하)': 'Z (up/down)',
    'Yaw (좌우)': 'Yaw',
    'Roll (기울기)': 'Roll',
    'Pitch (숙임)': 'Pitch',
    '키보드: 누르는 동안 이동 · 떼면 정지': 'Keyboard: hold to move · release to stop',
    '좌측 이동': 'Strafe L',
    '우측 이동': 'Strafe R',
    '좌회전': 'Turn L',
    '우회전': 'Turn R',
    '손 제어': 'Hand control',
    '서버 연결 대기 중...': 'Waiting for server...',
    '손 펴기 (unfold)': 'Open hand (unfold)',
    '타임라인 에디터': 'Timeline editor',
    '프레임 시간 (초)': 'Frame time (s)',
    '이동 명령': 'Move command',
    '손 모션': 'Hand motion',
    '손 선택': 'Hand',
    '현재 자세 저장': 'Save current pose',
    '포함 (현재 모드 기준)': 'Include (current mode)',
    '포함 (팔 + 허리)': 'Include (arm + waist)',
    '포함 (XYZ + Rotation)': 'Include (XYZ + Rotation)',
    '안전 모드 (Joint)': 'Safe mode (Joint)',
    'x0.5 제한': 'x0.5 limit',
    '프레임 추가': 'Add frame',
    '불러오기': 'Load',
    '파일 이름:': 'File name:',
    '파일 오류': 'File error',

    // ===== 공통 단어 (앞뒤가 한글이 아닐 때만 바뀜) =====
    '연결 끊김': 'disconnected',
    '확인 불가': 'unknown',
    '확인 중': 'checking',
    '실행 중': 'running',
    '전송 중': 'sending',
    '전환 중': 'switching',
    '추종 중': 'Following',
    '시간 초과': 'Timeout',
    '정지됨': 'Stopped',
    '취소됨': 'cancelled',
    '미연결': 'not connected',
    '연결됨': 'connected',
    '오른손': 'Right hand',
    '왼손': 'Left hand',
    '양손': 'Both',
    '경고': 'Warning',
    '대기': 'Idle',
    '취소': 'Cancel',
    '실행': 'Run',
    '정지': 'Stop',
    '거부': 'rejected',
    '실패': 'failed',
    '오류': 'error',
    '종료': 'exit',
    '완료': 'done',
    '중단': 'aborted',
    '도착': 'Arrived',
    '최근': 'recent',
    '연결': 'Link',
    '작업': 'Task',
    '걷기': 'walk',
    '추론': 'Inference',
    '신뢰도': 'Confidence',
    '거리': 'Distance',
    '높이': 'H',
    '기본': 'Default',
    '부하': 'Load',
    '자동 회전': 'Auto orbit',
    '천천히 좌우로 회전 (마우스 조작 시 멈추고 5s 후 재개)': 'Slow side-to-side orbit (pauses on mouse input, resumes after 5 s)',
    '끊김': 'lost',
    '현재': 'now',
    '에서만': 'only',
    '이동': 'Move',
    '전진': 'Forward',
    '후진': 'Back',
    '좌측': 'Left',
    '우측': 'Right',
    '없음': 'None',
    '포함': 'Include',
    '유형': 'Type',
    '내용': 'Content',
    '시간': 'Time',
    '재생': 'Play',
    '저장': 'Save',
    '자세': 'pose',
    '허리': 'waist',
    '팔': 'arm',
  };

  // 숫자가 들어간 문구 (사전보다 먼저)
  const RULES = [
    [/FSM 은 (\([^)]*\)) 중 하나/g, 'FSM must be one of $1'],
    [/(\d+)초 후 전송/g, 'sending in $1s'],
    [/(\d+)분 (\d+)초/g, '$1m $2s'],
    [/(\d+(?:\.\d+)?)초 이상 미검출/g, 'not seen for $1s+'],
    [/(\d+(?:\.\d+)?)초 초과/g, 'over $1s'],
    [/(\d+)개 서버/g, '$1 servers'],
    [/(\d+(?:\.\d+)?)초/g, '$1s'],
  ];

  const HANGUL = /[가-힣]/;
  const esc = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  const PHRASES = Object.keys(DICT).sort((a, b) => b.length - a.length)
    .map(k => [new RegExp('(?<![가-힣])' + esc(k) + '(?![가-힣])', 'g'), DICT[k]]);

  function tr(s) {
    if (!s || !HANGUL.test(s)) return s;
    for (const [re, to] of RULES) s = s.replace(re, to);
    for (const [re, to] of PHRASES) { if (!HANGUL.test(s)) break; s = s.replace(re, to); }
    return s;
  }

  // ---------- 상태 ----------
  const q = new URLSearchParams(location.search);
  let lang = q.get('lang');
  if (lang !== 'en' && lang !== 'ko') {
    try { lang = localStorage.getItem('g1_lang'); } catch (e) { lang = null; }
  }
  if (lang !== 'en') lang = 'ko';
  const embed = q.get('embed') === '1';

  const ORIG = '__g1ko', DONE = '__g1en';
  const ATTRS = ['title', 'placeholder'];

  function trNode(n) {
    if (n.nodeType === 3) {                         // 텍스트
      const p = n.parentNode;
      if (p && (p.nodeName === 'SCRIPT' || p.nodeName === 'STYLE')) return;
      const v = n.nodeValue;
      if (lang === 'en') {
        if (HANGUL.test(v)) { const t = tr(v); if (t !== v) { n[ORIG] = v; n[DONE] = t; n.nodeValue = t; } }
      } else if (n[ORIG] !== undefined && v === n[DONE]) {
        n.nodeValue = n[ORIG]; delete n[ORIG]; delete n[DONE];
      }
    } else if (n.nodeType === 1) {                  // 요소 속성
      for (const a of ATTRS) {
        const v = n.getAttribute && n.getAttribute(a);
        if (v == null) continue;
        const ko = '__g1ko_' + a, en = '__g1en_' + a;
        if (lang === 'en') {
          if (HANGUL.test(v)) { const t = tr(v); if (t !== v) { n[ko] = v; n[en] = t; n.setAttribute(a, t); } }
        } else if (n[ko] !== undefined && v === n[en]) { n.setAttribute(a, n[ko]); delete n[ko]; delete n[en]; }
      }
    }
  }
  function walk(root) {
    if (!root) return;
    trNode(root);
    const it = document.createTreeWalker(root, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
    let n; while ((n = it.nextNode())) trNode(n);
  }

  let busy = false;
  const mo = new MutationObserver(muts => {
    if (busy || lang !== 'en') return;
    busy = true;
    try {
      for (const m of muts) {
        if (m.type === 'characterData') trNode(m.target);
        else if (m.type === 'attributes') trNode(m.target);
        else m.addedNodes.forEach(walk);
      }
    } finally { busy = false; }
  });

  // alert / confirm / prompt 문구
  const _alert = window.alert.bind(window), _confirm = window.confirm.bind(window), _prompt = window.prompt.bind(window);
  window.alert = m => _alert(lang === 'en' ? tr(String(m)) : m);
  window.confirm = m => _confirm(lang === 'en' ? tr(String(m)) : m);
  window.prompt = (m, d) => _prompt(lang === 'en' ? tr(String(m)) : m, d);

  // ---------- 버튼 ----------
  function renderBtn() {
    if (embed) return;
    let b = document.getElementById('g1-lang-btn');
    if (!b) {
      b = document.createElement('button');
      b.id = 'g1-lang-btn'; b.type = 'button';
      b.title = 'Language / 언어';
      b.style.cssText = 'font:600 11px/1 ui-monospace,Menlo,monospace;border-radius:999px;cursor:pointer;' +
        'padding:5px 10px;background:#1c232d;border:1px solid #2a3340;color:#c9d4e0;letter-spacing:.5px;white-space:nowrap';
      b.addEventListener('click', () => setLang(lang === 'en' ? 'ko' : 'en'));
      const slot = document.getElementById('lang-slot');
      if (slot) slot.appendChild(b);
      else { b.style.position = 'fixed'; b.style.right = '12px'; b.style.bottom = '12px'; b.style.zIndex = '9999'; document.body.appendChild(b); }
    }
    b.innerHTML = lang === 'en'
      ? '<span style="opacity:.45">한</span> | <b style="color:#4aa8ff">EN</b>'
      : '<b style="color:#4aa8ff">한</b> | <span style="opacity:.45">EN</span>';
  }

  let domReady = document.readyState !== 'loading';
  function setLang(l, fromParent) {
    lang = l === 'en' ? 'en' : 'ko';
    try { localStorage.setItem('g1_lang', lang); } catch (e) {}
    document.documentElement.lang = lang;
    busy = true; try { walk(document.body); } finally { busy = false; }
    if (domReady) renderBtn();          // 버튼 자리(#lang-slot)가 파싱된 뒤에 그린다
    // iframe 에도 전달 (포트가 다르면 localStorage 공유가 안 됨)
    if (!fromParent) document.querySelectorAll('iframe').forEach(f => {
      try { f.contentWindow.postMessage({ g1lang: lang }, '*'); } catch (e) {}
    });
    window.dispatchEvent(new CustomEvent('g1lang', { detail: lang }));
  }
  window.addEventListener('message', e => {
    if (e.data && (e.data.g1lang === 'en' || e.data.g1lang === 'ko') && e.data.g1lang !== lang) setLang(e.data.g1lang, true);
  });

  window.G1I18N = { get lang() { return lang; }, setLang, tr };

  function start() {
    mo.observe(document.body, { childList: true, subtree: true, characterData: true, attributes: true, attributeFilter: ATTRS });
    setLang(lang, true);
    if (!domReady) document.addEventListener('DOMContentLoaded', () => { domReady = true; renderBtn(); });
  }
  if (document.body) start(); else document.addEventListener('DOMContentLoaded', start);
})();

// ═══════════════════════════════════════════════════════════════
//  MantisSec CSI-Radar-S3 web flasher — client logic
//
//  Wires the two dropdowns (device role + board) to the ESP Web Tools
//  install button.  When both are selected, updates the button's
//  manifest attribute to point at the correct .json in ./manifests/.
//  Also handles the browser gate (Web Serial requires Chromium-family)
//  and translates ESP Web Tools state events into MantisSec-styled
//  status lines.
// ═══════════════════════════════════════════════════════════════

(function () {
    'use strict';

    // ── Elements ──
    const roleSel   = document.getElementById('role-select');
    const boardSel  = document.getElementById('board-select');
    const btn       = document.getElementById('install-btn');
    const btnLabel  = document.getElementById('btn-label');
    const status    = document.getElementById('status');
    const statusMsg = document.getElementById('status-msg');
    const browserWarn = document.getElementById('browser-warn');

    // ── Manifest map ──
    // Key = "<role>|<board>", value = manifest URL.
    // Board is informational — the actual chip family is auto-detected
    // by ESP Web Tools at connect time.  The map exists so we can gate
    // the flash button on both dropdowns being set.
    const MANIFESTS = {
        'receiver|t-display-s3':   'manifests/receiver.json',
        'beacon-1|xiao-c3':         'manifests/beacon-1.json',
        'beacon-1|xiao-s3':         'manifests/beacon-1.json',
        'beacon-2|xiao-c3':         'manifests/beacon-2.json',
        'beacon-2|xiao-s3':         'manifests/beacon-2.json',
        'beacon-3|xiao-c3':         'manifests/beacon-3.json',
        'beacon-3|xiao-s3':         'manifests/beacon-3.json',
    };

    // Which boards go with which role
    const ROLE_BOARDS = {
        'receiver': [
            { value: 't-display-s3', label: 'LilyGo T-Display S3' }
        ],
        'beacon-1': [
            { value: 'xiao-c3', label: 'Seeed XIAO ESP32-C3' },
            { value: 'xiao-s3', label: 'Seeed XIAO ESP32-S3' }
        ],
        'beacon-2': [
            { value: 'xiao-c3', label: 'Seeed XIAO ESP32-C3' },
            { value: 'xiao-s3', label: 'Seeed XIAO ESP32-S3' }
        ],
        'beacon-3': [
            { value: 'xiao-c3', label: 'Seeed XIAO ESP32-C3' },
            { value: 'xiao-s3', label: 'Seeed XIAO ESP32-S3' }
        ],
    };

    // ── Status helpers ──
    function setStatus(kind, msg) {
        status.className = 'status ' + kind;
        statusMsg.textContent = msg;
    }

    // ── Browser gate ──
    // Web Serial API is required.  Chromium-family only (Chrome, Edge,
    // Opera, Brave, Vivaldi on desktop).  Safari, Firefox, mobile: nope.
    function checkBrowser() {
        if (!('serial' in navigator)) {
            browserWarn.classList.add('show');
            browserWarn.innerHTML =
                '<strong>Browser not supported.</strong> ' +
                'The web flasher needs the Web Serial API. ' +
                'Please open this page in <strong>Chrome</strong>, ' +
                '<strong>Edge</strong>, or another Chromium-based ' +
                'browser on a <strong>desktop or laptop</strong>. ' +
                'iOS and most mobile browsers cannot flash USB devices.';
            roleSel.disabled = true;
            boardSel.disabled = true;
            btn.classList.remove('ready');
            btnLabel.textContent = 'Browser not supported';
            setStatus('error', 'Web Serial API missing from this browser.');
            return false;
        }
        return true;
    }

    // ── Dropdown logic ──
    function populateBoards() {
        boardSel.innerHTML = '';
        const role = roleSel.value;
        if (!role) {
            const opt = document.createElement('option');
            opt.value = '';
            opt.textContent = '— select device first —';
            boardSel.appendChild(opt);
            boardSel.disabled = true;
            updateButton();
            return;
        }
        const boards = ROLE_BOARDS[role] || [];
        const placeholder = document.createElement('option');
        placeholder.value = '';
        placeholder.textContent = boards.length > 1
            ? '— pick which XIAO you have —'
            : '';
        if (boards.length > 1) boardSel.appendChild(placeholder);
        for (const b of boards) {
            const opt = document.createElement('option');
            opt.value = b.value;
            opt.textContent = b.label;
            boardSel.appendChild(opt);
        }
        // If only one board, auto-select
        if (boards.length === 1) {
            boardSel.value = boards[0].value;
        }
        boardSel.disabled = false;
        updateButton();
    }

    function updateButton() {
        const key = `${roleSel.value}|${boardSel.value}`;
        const manifest = MANIFESTS[key];
        if (!manifest) {
            btn.classList.remove('ready');
            btn.removeAttribute('manifest');
            btnLabel.textContent = 'Select device & board';
            setStatus('info',
                'Pick what you\'re flashing and which board you have plugged in.');
            return;
        }
        btn.classList.add('ready');
        btn.setAttribute('manifest', manifest);
        const roleLabel = roleSel.options[roleSel.selectedIndex].textContent;
        btnLabel.textContent = `Connect & Flash — ${roleLabel}`;
        setStatus('info',
            `Ready. Plug the device into USB, then click the button above. ` +
            `Your browser will show a port picker; choose the ESP32 port.`);
    }

    // ── ESP Web Tools state events ──
    // See https://esphome.github.io/esp-web-tools/ for the event names.
    function wireEwt() {
        btn.addEventListener('state-changed', (ev) => {
            const s = ev.detail;
            if (!s) return;

            // s.state values: initializing, manifest, preparing, erasing,
            //   writing, finished, error
            switch (s.state) {
                case 'initializing':
                    setStatus('working', 'Initializing connection…');
                    break;
                case 'manifest':
                    setStatus('working', 'Fetched manifest — verifying chip…');
                    break;
                case 'preparing':
                    setStatus('working',
                        s.message || 'Preparing device for flashing…');
                    break;
                case 'erasing':
                    setStatus('working',
                        s.message || 'Erasing existing firmware…');
                    break;
                case 'writing':
                    if (typeof s.details?.percentage === 'number') {
                        setStatus('working',
                            `Writing firmware — ${Math.round(s.details.percentage)}%`);
                    } else {
                        setStatus('working', s.message || 'Writing firmware…');
                    }
                    break;
                case 'finished':
                    setStatus('good',
                        'Done. Firmware installed successfully. ' +
                        'Unplug the device and plug it back in to boot.');
                    break;
                case 'error':
                    setStatus('error',
                        (s.message || 'Flashing failed.') +
                        ' — Try again, or hold the BOOT button while plugging in.');
                    break;
                default:
                    if (s.message) setStatus('info', s.message);
            }
        });
    }

    // ── Init ──
    document.addEventListener('DOMContentLoaded', () => {
        // Populate role options are static in HTML; wire change handlers
        roleSel.addEventListener('change', populateBoards);
        boardSel.addEventListener('change', updateButton);
        populateBoards();
        if (checkBrowser()) {
            wireEwt();
        }
    });
})();

// ═══════════════════════════════════════════════════════════════
//  MantisSec — overlays.js
//  Extends the hero animation pattern to the other figures.
//
//  Same approach as the hero overlay in index.html: the PNG stays the
//  base layer untouched, a transparent canvas rides on top with
//  pointer-events:none, and everything is drawn with
//  globalCompositeOperation='screen' so the overlay only ever ADDS
//  light.  It cannot darken or obscure the artwork underneath.
//
//  Nothing here modifies any image, any copy, or any existing script.
//  Drop in with one line at the bottom of index.html:
//      <script src="overlays.js"></script>
//
//  It deliberately does NOT touch .lp-hero-img — that one already has
//  its own engine and is left exactly as it is.
// ═══════════════════════════════════════════════════════════════

(() => {
"use strict";

// ── Palette ──────────────────────────────────────────────────────
// The three MantisSec colours: teal #007373, violet #5D005D, lime
// #A8FF00, plus the -lo/-hi variants from style.css.  Nothing here
// invents a colour; these are the same values the stylesheet declares.
const C = {
    teal:     'rgba(0, 115, 115, ',    // --ms-teal    #007373
    tealLo:   'rgba(0, 165, 165, ',    // --ms-teal-lo #00A5A5
    tealHi:   'rgba(51, 214, 214, ',   // --ms-teal-hi #33d6d6
    violet:   'rgba(93, 0, 93, ',      // --ms-violet  #5D005D
    violetLo: 'rgba(143, 0, 165, ',    // --ms-violet-lo #8F00A5
    violetHi: 'rgba(192, 0, 224, ',    // --ms-violet-hi #c000e0
    lime:     'rgba(168, 255, 0, ',    // --ms-lime    #A8FF00
};

// ═══════════════════════════════════════════════════════════════
//  TUNING — anchor points, as fractions of each image's width and
//  height so they track at any size.
//
//  These were read off the current artwork by eye.  If a figure ever
//  gets re-exported and the overlay drifts, these numbers are the only
//  thing that needs nudging — nothing below depends on absolute pixels.
// ═══════════════════════════════════════════════════════════════
const TUNE = {
    how_it_works: {
        // Each panel is a SELF-CONTAINED scene.  Panel 2 has its own two
        // beacons on its own side walls -- it is a different room drawn
        // from a different angle, not the same room as panel 1.  Reusing
        // panel 1's beacons here is what made the scatter paths shoot
        // across the gutter.  Each panel also carries its card bounds so
        // its drawing can be clipped to itself.
        place: {
            clip:    [0.012, 0.326],
            beacons: [ [0.053, 0.386], [0.188, 0.330], [0.293, 0.380] ],
            rx:        [0.180, 0.553],
        },
        calibrate: {
            clip:    [0.351, 0.665],
            beacons: [ [0.393, 0.367], [0.632, 0.372] ],   // its own, two of them
            rx:      [0.496, 0.645],
            centre:  [0.503, 0.517],
            rx_norm: 0.104,
            ry_norm: 0.076,
        },
        solve: {
            clip:    [0.693, 0.991],
            targets: [ [0.849, 0.384], [0.783, 0.503], [0.909, 0.499] ],
        },
    },
    observe_live: {
        centre: [0.500, 0.520],
        radius: 0.360,
    },
};

// ═══════════════════════════════════════════════════════════════
//  MOUNT — generalised version of the hero's initInteractiveOverlay
// ═══════════════════════════════════════════════════════════════
function mountOverlay(imgElement, drawFn) {
    // Wrap the image so the canvas can be absolutely positioned against
    // it without disturbing the surrounding layout or its margins.
    const container = document.createElement('div');
    container.style.position = 'relative';
    container.style.display  = 'block';
    container.style.width    = '100%';
    imgElement.parentNode.insertBefore(container, imgElement);
    container.appendChild(imgElement);

    const canvas = document.createElement('canvas');
    canvas.style.position      = 'absolute';
    canvas.style.pointerEvents = 'none';   // image stays right-clickable
    container.appendChild(canvas);
    const ctx = canvas.getContext('2d');

    // Match the backing store to the displayed size, accounting for
    // device pixel ratio so the strokes stay crisp on retina panels.
    function synchronizeBounds() {
        const dpr = window.devicePixelRatio || 1;
        const w = imgElement.clientWidth;
        const h = imgElement.clientHeight;
        if (!w || !h) return;
        // Track the IMAGE's box, not the wrapper's.  Several figures on
        // this site are narrower than their container and centred with
        // `margin: auto` (.lp-fullwidth-img is max-width 720px), so a
        // canvas sized to the wrapper would overhang the artwork and put
        // every overlay coordinate in the wrong place on wide viewports.
        canvas.style.left   = imgElement.offsetLeft + 'px';
        canvas.style.top    = imgElement.offsetTop  + 'px';
        canvas.style.width  = w + 'px';
        canvas.style.height = h + 'px';
        canvas.width  = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    // The image may finish loading or reflow after mount; keep re-checking
    // cheaply for the first couple of seconds rather than trusting one shot.
    let syncTicks = 0;
    const syncTimer = setInterval(() => {
        synchronizeBounds();
        if (++syncTicks > 20) clearInterval(syncTimer);
    }, 100);
    window.addEventListener('resize', synchronizeBounds);
    imgElement.addEventListener('load', synchronizeBounds);
    if (imgElement.complete) synchronizeBounds();

    // Only run while the figure is actually on screen.  These loops are
    // cheap individually but there is no reason to burn a phone's
    // battery animating a diagram three screens further down the page.
    let visible = false;
    if ('IntersectionObserver' in window) {
        new IntersectionObserver((entries) => {
            visible = entries[0].isIntersecting;
        }, { threshold: 0.05 }).observe(container);
    } else {
        visible = true;
    }

    let frame = 0;
    function engine() {
        if (visible && canvas.width) {
            frame++;
            const w = canvas.width  / (window.devicePixelRatio || 1);
            const h = canvas.height / (window.devicePixelRatio || 1);
            ctx.clearRect(0, 0, w, h);
            ctx.globalCompositeOperation = 'screen';
            drawFn(ctx, w, h, frame);
        }
        requestAnimationFrame(engine);
    }
    requestAnimationFrame(engine);
}

// ═══════════════════════════════════════════════════════════════
//  HOW IT WORKS — one motion per panel, each showing that step's job
// ═══════════════════════════════════════════════════════════════
function drawHowItWorks(ctx, w, h, frame) {
    const T = TUNE.how_it_works;

    // Run a panel's drawing clipped to that panel's card.  Structural,
    // not cosmetic: it makes it impossible for one panel's geometry to
    // reach into another regardless of what the draw code does.
    function inPanel(cfg, fn) {
        ctx.save();
        ctx.beginPath();
        ctx.rect(cfg.clip[0] * w, 0, (cfg.clip[1] - cfg.clip[0]) * w, h);
        ctx.clip();
        fn();
        ctx.restore();
    }

    // ── Panel 1, PLACE: an empty room being surveyed ──
    inPanel(T.place, () => {
        T.place.beacons.forEach((b, i) => {
            const cycle = ((frame + i * 40) % 170) / 170;
            ctx.strokeStyle = `${C.violetHi}${0.30 * (1 - cycle)})`;
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.arc(b[0] * w, b[1] * h, cycle * (w * 0.070), 0, Math.PI * 2);
            ctx.stroke();
        });
        T.place.beacons.forEach((b, i) => {
            const pulse = 0.10 + 0.16 * Math.max(0, Math.sin(frame * 0.045 - i * 1.9));
            ctx.strokeStyle = `${C.tealLo}${pulse})`;
            ctx.lineWidth = 0.9;
            ctx.beginPath();
            ctx.moveTo(b[0] * w, b[1] * h);
            ctx.lineTo(T.place.rx[0] * w, T.place.rx[1] * h);
            ctx.stroke();
        });
    });

    // ── Panel 2, CALIBRATE: the walk, using THIS panel's geometry ──
    //
    // The hero bends a scatter path through a stationary body.  Here the
    // body is moving, so both beacon->body->RX paths re-route every frame
    // and the distortion envelope travels with the walker.  That is what
    // the walk cal records: at each position on the loop, what every
    // beacon's path looks like with a person standing there.
    inPanel(T.calibrate, () => {
        const cal = T.calibrate;
        const cx = cal.centre[0] * w, cy = cal.centre[1] * h;
        const erx = cal.rx_norm * w,  ery = cal.ry_norm * h;
        const theta = (frame * 0.013) % (Math.PI * 2);
        const wx = cx + erx * Math.cos(theta);
        const wy = cy + ery * Math.sin(theta);
        const rxx = cal.rx[0] * w, rxy = cal.rx[1] * h;

        ctx.strokeStyle = `${C.lime}0.10)`;
        ctx.lineWidth = 0.8;
        ctx.beginPath();
        ctx.ellipse(cx, cy, erx, ery, 0, 0, Math.PI * 2);
        ctx.stroke();

        cal.beacons.forEach((b, i) => {
            const bx = b[0] * w, by = b[1] * h;
            ctx.strokeStyle = `${C.lime}0.30)`;
            ctx.lineWidth = 1.1;
            ctx.beginPath();
            const segments = 40;
            for (let k = 0; k <= segments; k++) {
                const pct = k / segments;
                let px, py;
                if (pct < 0.5) {
                    const t = pct * 2;
                    px = bx + (wx - bx) * t;
                    py = by + (wy - by) * t;
                } else {
                    const t = (pct - 0.5) * 2;
                    px = wx + (rxx - wx) * t;
                    py = wy + (rxy - wy) * t;
                }
                const nearBody = Math.max(0, 1 - Math.abs(pct - 0.5) * 3.2);
                const dist = Math.sin(pct * 20 - frame * 0.16 + i * 2.1)
                           * (3.0 * nearBody);
                if (k === 0) ctx.moveTo(px, py + dist);
                else         ctx.lineTo(px, py + dist);
            }
            ctx.stroke();
        });

        const TRAIL = 22;
        for (let k = TRAIL; k >= 0; k--) {
            const t = theta - k * 0.05;
            const px = cx + erx * Math.cos(t);
            const py = cy + ery * Math.sin(t);
            const fade = 1 - k / TRAIL;
            ctx.fillStyle = `${C.lime}${0.45 * fade * fade})`;
            ctx.beginPath();
            ctx.arc(px, py, 1.2 + 2.2 * fade, 0, Math.PI * 2);
            ctx.fill();
        }
    });

    // ── Panel 3, SOLVE: posteriors contracting onto their fixes ──
    inPanel(T.solve, () => {
        T.solve.targets.forEach((p, i) => {
            const px = p[0] * w, py = p[1] * h;
            const phase = ((frame * 0.006) + i * 0.33) % 1;
            const k = phase < 0.7 ? (1 - phase / 0.7) : 0;
            const eased = k * k;
            const rWide = w * 0.048, rTight = w * 0.015;
            const r = rTight + (rWide - rTight) * eased;
            ctx.strokeStyle = `${C.lime}${0.10 + 0.22 * (1 - eased)})`;
            ctx.lineWidth = 0.9;
            ctx.beginPath();
            ctx.arc(px, py, r, 0, Math.PI * 2);
            ctx.stroke();

            const breath = Math.sin(frame * 0.035 + i * 2.1);
            ctx.strokeStyle = `${C.lime}0.26)`;
            ctx.lineWidth = 0.8;
            ctx.beginPath();
            ctx.arc(px, py, rTight + breath * (w * 0.002), 0, Math.PI * 2);
            ctx.stroke();
        });
    });
}

// ═══════════════════════════════════════════════════════════════
//  LIVE VIEW — a slow sweep over the radar screencap
// ═══════════════════════════════════════════════════════════════
function drawLiveSweep(ctx, w, h, frame) {
    const T = TUNE.observe_live;
    const cx = T.centre[0] * w, cy = T.centre[1] * h;
    const R  = T.radius * Math.min(w, h);
    const a  = (frame * 0.012) % (Math.PI * 2);

    // Sweep wedge, trailing behind the leading edge so it reads as a
    // direction of travel rather than a spinning line.
    const WEDGE = 0.55;
    const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, R);
    grad.addColorStop(0, `${C.tealHi}0.00)`);
    grad.addColorStop(1, `${C.tealHi}0.13)`);
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, R, a - WEDGE, a);
    ctx.closePath();
    ctx.fill();

    // Leading edge
    ctx.strokeStyle = `${C.tealHi}0.30)`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + R * Math.cos(a), cy + R * Math.sin(a));
    ctx.stroke();

    // One slow range ring, so the sweep has something to sweep over.
    const ringCycle = (frame % 260) / 260;
    ctx.strokeStyle = `${C.tealLo}${0.16 * (1 - ringCycle)})`;
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    ctx.arc(cx, cy, ringCycle * R, 0, Math.PI * 2);
    ctx.stroke();
}

// ═══════════════════════════════════════════════════════════════
//  REGISTRY
// ═══════════════════════════════════════════════════════════════
const OVERLAYS = [
    { selector: 'img[data-asset="how_it_works"]', draw: drawHowItWorks },
    { selector: 'img[data-asset="observe_live"]', draw: drawLiveSweep  },
];

document.addEventListener('DOMContentLoaded', () => {
    // Anyone who has asked their OS for less animation gets the static
    // artwork, which already carries the full message on its own.
    const reduced = window.matchMedia &&
                    window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (reduced) return;

    OVERLAYS.forEach(o => {
        const el = document.querySelector(o.selector);
        if (el) mountOverlay(el, o.draw);
    });
});

})();

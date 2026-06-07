// ==========================================
// app_231_012.js
// ==========================================
const T20_CONST = {
    API_BASE: "/api/t20",
    WS: {
        RECONNECT_MS: 2000,
        PROTOCOL: window.location.protocol === 'https:' ? 'wss:' : 'ws:',
    },
    CHART_LIMITS: {
        WAVE: { MIN: -2.0, MAX: 2.0 },
        SPEC: { MIN: 0.0, MAX: 15.0 },
        MFCC_STATIC: { MIN: -20.0, MAX: 20.0 },
        MFCC_DELTA: { MIN: -5.0, MAX: 5.0 },
        MFCC_ACCEL: { MIN: -2.0, MAX: 2.0 }
    },
    UI: { TOAST_MS: 3000, DIAG_INTERVAL_MS: 1000, REBOOT_RELOAD_MS: 6000, MAX_ROUTERS: 3 }
};

let sysState = {
    fft_size: 256,
    axis_count: 1,
    mfcc_coeffs: 13,
    sequence_frames: 16
};

let socket;
let charts = {};
let isLearning = false;
let diagTimer = null;

let pendingWave = null;
let pendingSpec = null;
let pendingMfcc = null;
let pendingSeqFrames = [];
let isSinglePending = false;
let isSeqPending = false;

// ==========================================
// 1. UI 및 유틸리티 로직
// ==========================================
function showTab(tabId, e = null) {
    document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(tabId).classList.add('active');

    if(e && e.currentTarget) e.currentTarget.classList.add('active');

    if (tabId === 'tab-diag') {
        fetchDiagnostics();
        diagTimer = setInterval(fetchDiagnostics, T20_CONST.UI.DIAG_INTERVAL_MS);
    } else if (diagTimer) {
        clearInterval(diagTimer);
    }
}

function showToast(msg, isError = false) {
    const toast = document.getElementById("toast");
    toast.innerText = msg;
    toast.className = isError ? "toast show error" : "toast show";
    setTimeout(() => { toast.className = toast.className.replace("show", ""); }, T20_CONST.UI.TOAST_MS);
}

function toggleWifiFields() {
    const mode = document.querySelector('[name="wifi.mode"]').value;
    document.getElementById("section_ap_config").style.display = (mode === "0") ? "none" : "block";
    document.getElementById("section_sta_config").style.display = (mode === "1") ? "none" : "block";
}

// [정렬 보정] 공유기 설정 HTML 렌더링 시, 좁은 폭에서도 깨지지 않도록 flex-wrap: wrap 적용
function renderRouterHTML(idx) {
    return `
    <div style="border-bottom: 1px solid #444; padding-bottom: 10px; margin-bottom: 10px;">
        <label style="color:#aaa; font-size:0.85em; display:block; margin-bottom:6px;">Router ${idx + 1}</label>
        <div style="display:flex; flex-wrap:wrap; gap:8px; margin-bottom:8px;">
            <input type="text" name="multi_ssid_${idx}" placeholder="SSID" style="flex: 1 1 40%; min-width: 120px;">
            <input type="password" name="multi_pass_${idx}" placeholder="Password" style="flex: 1 1 40%; min-width: 120px;">
            <select name="multi_static_${idx}" onchange="document.getElementById('rip_${idx}').style.display = this.value === 'true' ? 'grid' : 'none'" style="flex: 1 1 100%;">
                <option value="false">DHCP</option><option value="true">Static</option>
            </select>
        </div>
        <div id="rip_${idx}" style="display:none; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap:8px;">
             <input type="text" name="multi_ip_${idx}" placeholder="IP" style="width:100%;">
             <input type="text" name="multi_gw_${idx}" placeholder="Gateway" style="width:100%;">
             <input type="text" name="multi_sn_${idx}" placeholder="Subnet" style="width:100%;">
             <input type="text" name="multi_dns1_${idx}" placeholder="DNS 1" style="width:100%;">
             <input type="text" name="multi_dns2_${idx}" placeholder="DNS 2" style="width:100%;">
        </div>
    </div>`;
}

function initUI() {
    const container = document.getElementById('routers_container');
    if(container) { 
        let html = "";
        for(let i=0; i < T20_CONST.UI.MAX_ROUTERS; i++) html += renderRouterHTML(i);
        container.innerHTML = html;
    }
    toggleWifiFields();
}

// ==========================================
// 2. 동적 차트 엔진 제어
// ==========================================
function getChartOptions(titleText, xLabel, yLabel, yMin, yMax) {
    return {
        animation: false, responsive: true, maintainAspectRatio: false,
        plugins: {
            legend: { display: false },
            title: { display: true, text: titleText, color: '#ccc', font: { size: 14 } }
        },
        elements: { point: { radius: 0 } },
        scales: {
            x: { title: { display: true, text: xLabel, color: '#888' }, ticks: { color: '#666' } },
            y: { title: { display: true, text: yLabel, color: '#888' }, ticks: { color: '#666' }, min: yMin, max: yMax }
        }
    };
}

function initCharts() {
    charts.wave = new Chart(document.getElementById('chart-waveform'), { 
        type: 'line', data: { labels: [], datasets: [{ data: [], borderColor: '#4caf50', borderWidth: 1 }] }, 
        options: getChartOptions('Raw Waveform (Time Domain)', 'Samples', 'Amplitude (G)', T20_CONST.CHART_LIMITS.WAVE.MIN, T20_CONST.CHART_LIMITS.WAVE.MAX)
    });

    charts.spec = new Chart(document.getElementById('chart-spectrum'), { 
        type: 'line', data: { labels: [], datasets: [{ data: [], borderColor: '#ff9800', borderWidth: 1, fill: true, backgroundColor: 'rgba(255,152,0,0.1)' }] }, 
        options: getChartOptions('Power Spectrum (Freq Domain)', 'Frequency Bins', 'Power Energy', T20_CONST.CHART_LIMITS.SPEC.MIN, T20_CONST.CHART_LIMITS.SPEC.MAX)
    });

    charts.mfcc = new Chart(document.getElementById('chart-mfcc'), { 
        type: 'bar', data: { labels: [], datasets: [{ data: [], backgroundColor: [] }] }, 
        options: getChartOptions('MFCC Features', 'Coefficients (C, D, DD)', 'Magnitude', null, null)
    });
}

function updateChartDimensions() {
    const N = sysState.fft_size;
    const Bins = Math.floor(N / 2) + 1;
    const mfccTotal = sysState.mfcc_coeffs * 3 * sysState.axis_count;

    charts.wave.data.labels = Array(N).fill('');
    charts.spec.data.labels = Array(Bins).fill('');
    charts.mfcc.data.labels = Array(mfccTotal).fill('');
    
    charts.wave.data.datasets = [];
    charts.spec.data.datasets = [];
    const lineColors = ['#4caf50', '#ff9800', '#f44336'];
    const bgColors = ['rgba(76, 175, 80, 0.1)', 'rgba(255, 152, 0, 0.1)', 'rgba(244, 67, 54, 0.1)'];

    for(let i=0; i < sysState.axis_count; i++) {
        charts.wave.data.datasets.push({ data: [], borderColor: lineColors[i], borderWidth: 1, pointRadius: 0 });
        charts.spec.data.datasets.push({ data: [], borderColor: lineColors[i], borderWidth: 1, fill: true, backgroundColor: bgColors[i], pointRadius: 0 });
    }
    
    const mfccColors = [];
    for(let i=0; i < mfccTotal; i++) {
        const axisIdx = Math.floor(i / (sysState.mfcc_coeffs * 3));
        const featType = Math.floor((i % (sysState.mfcc_coeffs * 3)) / sysState.mfcc_coeffs); 
        const alpha = featType === 0 ? '1.0' : (featType === 1 ? '0.7' : '0.4');
        let rgb = axisIdx === 0 ? '76, 175, 80' : (axisIdx === 1 ? '255, 152, 0' : '244, 67, 54');
        mfccColors.push(`rgba(${rgb}, ${alpha})`);
    }
    if (charts.mfcc.data.datasets.length === 0) charts.mfcc.data.datasets.push({ data: [], backgroundColor: [] });
    charts.mfcc.data.datasets[0].backgroundColor = mfccColors;
    
    charts.wave.update('none');
    charts.spec.update('none');
    charts.mfcc.update('none');

    waterfallSpec.resize(Bins);
    waterfallMfccStatic.resize(sysState.mfcc_coeffs);
    waterfallMfccDelta.resize(sysState.mfcc_coeffs);
    waterfallMfccAccel.resize(sysState.mfcc_coeffs);
}

class WaterfallChart {
    constructor(canvasId, minVal, maxVal) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas ? this.canvas.getContext('2d', { alpha: false, willReadFrequently: true }) : null;
        this.minVal = minVal; this.maxVal = maxVal;
        if (this.canvas) this.canvas.width = 600; 
    }
    resize(newLen) {
        if(this.canvas && this.canvas.height !== newLen) {
            this.canvas.height = newLen;
            this.ctx.fillStyle = '#000';
            this.ctx.fillRect(0,0, this.canvas.width, this.canvas.height);
        }
    }
    draw(dataArray) {
        if (!this.ctx || !dataArray) return;
        const w = this.canvas.width, h = this.canvas.height;
        this.ctx.drawImage(this.canvas, 1, 0, w - 1, h, 0, 0, w - 1, h);
        for (let y = 0; y < h; y++) {
            this.ctx.fillStyle = getHeatmapColor(dataArray[h - 1 - y], this.minVal, this.maxVal);
            this.ctx.fillRect(w - 1, y, 1, 1);
        }
    }
}

function getHeatmapColor(value, min, max) {
    let norm = (value - min) / (max - min);
    norm = Math.max(0, Math.min(1, norm));
    const r = Math.max(0, Math.min(255, Math.floor(255 * (1.5 - Math.abs(1 - 4 * (norm - 0.5))))));
    const g = Math.max(0, Math.min(255, Math.floor(255 * (1.5 - Math.abs(1 - 4 * (norm - 0.25))))));
    const b = Math.max(0, Math.min(255, Math.floor(255 * (1.5 - Math.abs(1 - 4 * norm)))));
    return `rgb(${r},${g},${b})`;
}

const waterfallSpec = new WaterfallChart('chart-waterfall-spec', T20_CONST.CHART_LIMITS.SPEC.MIN, T20_CONST.CHART_LIMITS.SPEC.MAX);
const waterfallMfccStatic = new WaterfallChart('chart-waterfall-mfcc-static', T20_CONST.CHART_LIMITS.MFCC_STATIC.MIN, T20_CONST.CHART_LIMITS.MFCC_STATIC.MAX);
const waterfallMfccDelta = new WaterfallChart('chart-waterfall-mfcc-delta', T20_CONST.CHART_LIMITS.MFCC_DELTA.MIN, T20_CONST.CHART_LIMITS.MFCC_DELTA.MAX);
const waterfallMfccAccel = new WaterfallChart('chart-waterfall-mfcc-deltadelta', T20_CONST.CHART_LIMITS.MFCC_ACCEL.MIN, T20_CONST.CHART_LIMITS.MFCC_ACCEL.MAX);

function drawMfccWaterfalls(axis0Mfcc) {
    const dim = sysState.mfcc_coeffs;
    waterfallMfccStatic.draw(axis0Mfcc.subarray(0, dim));
    waterfallMfccDelta.draw(axis0Mfcc.subarray(dim, dim * 2));
    waterfallMfccAccel.draw(axis0Mfcc.subarray(dim * 2, dim * 3));
}

// ==========================================
// 3. WebSocket 및 60Hz 렌더링 최적화 루프
// ==========================================
function renderLoop() {
    if (isSinglePending && pendingWave && pendingSpec && pendingMfcc) {
        const fftSize = sysState.fft_size;
        const step = fftSize <= 512 ? 1 : (fftSize <= 1024 ? 2 : 4);

        for (let a = 0; a < sysState.axis_count; a++) {
            const rawW = pendingWave[a];
            const rawS = pendingSpec[a];

            if (step === 1) {
                charts.wave.data.datasets[a].data = rawW;
                charts.spec.data.datasets[a].data = rawS;
            } else {
                const decimatedW = [];
                const decimatedS = [];
                for (let i = 0; i < rawW.length; i += step) decimatedW.push(rawW[i]);
                for (let i = 0; i < rawS.length; i += step) decimatedS.push(rawS[i]);
                
                charts.wave.data.datasets[a].data = decimatedW;
                charts.spec.data.datasets[a].data = decimatedS;
            }
        }

        charts.mfcc.data.datasets[0].data = pendingMfcc;

        charts.wave.update('none');
        charts.spec.update('none');
        charts.mfcc.update('none');

        waterfallSpec.draw(pendingSpec[0]);
        const singleAxisDim = sysState.mfcc_coeffs * 3;
        drawMfccWaterfalls(pendingMfcc.subarray(0, singleAxisDim));

        isSinglePending = false; 
    } 
    else if (isSeqPending && pendingMfcc) {
        charts.mfcc.data.datasets[0].data = pendingMfcc;
        charts.mfcc.update('none');

        pendingSeqFrames.forEach(frame => drawMfccWaterfalls(frame));
        pendingSeqFrames = [];
        isSeqPending = false;
    }
    requestAnimationFrame(renderLoop);
}

function connectWebSocket() {
    socket = new WebSocket(`${T20_CONST.WS.PROTOCOL}//${window.location.host}${T20_CONST.API_BASE}/ws`);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => { 
        const tag = document.getElementById('status-tag');
        tag.className = "tag online"; tag.innerText = "WS CONNECTED"; 
    };

    socket.onmessage = (event) => {
        if (!(event.data instanceof ArrayBuffer)) return;
        const floats = new Float32Array(event.data);
    
        const N = sysState.fft_size;
        const Bins = Math.floor(N / 2) + 1;
        const singleAxisDim = sysState.mfcc_coeffs * 3;
        const totalMfccDim = singleAxisDim * sysState.axis_count;
        
        const expectedSingle = (N * sysState.axis_count) + (Bins * sysState.axis_count) + totalMfccDim;
        const expectedSeq = sysState.sequence_frames * totalMfccDim;
    
        if (floats.length === expectedSingle) {
            pendingWave = [];
            pendingSpec = [];
            for(let i=0; i < sysState.axis_count; i++) {
                pendingWave.push(floats.subarray(i * N, (i + 1) * N));
                pendingSpec.push(floats.subarray((sysState.axis_count * N) + (i * Bins), (sysState.axis_count * N) + ((i + 1) * Bins)));
            }
            pendingMfcc = floats.subarray((sysState.axis_count * N) + (sysState.axis_count * Bins), expectedSingle);
            isSinglePending = true;
        }
        else if (floats.length === expectedSeq) {
            pendingMfcc = floats.subarray(expectedSeq - totalMfccDim, expectedSeq);
            pendingSeqFrames = [];
            for(let i=0; i < sysState.sequence_frames; i++) {
                pendingSeqFrames.push(floats.subarray(i * totalMfccDim, i * totalMfccDim + singleAxisDim));
            }
            isSeqPending = true;
        }
    };

    socket.onclose = () => { 
        const tag = document.getElementById('status-tag');
        tag.className = "tag offline"; tag.innerText = "WS DISCONNECTED"; 
        setTimeout(connectWebSocket, T20_CONST.WS.RECONNECT_MS); 
    };
}

// ==========================================
// 4. API 제어 및 업데이트 기능
// ==========================================
const callAPI = (path, method = 'POST') => fetch(`${T20_CONST.API_BASE}${path}`, { method });

document.getElementById('btn-start').onclick = () => callAPI('/recorder_begin').then(() => showToast("Measurement Started"));
document.getElementById('btn-stop').onclick = () => callAPI('/recorder_end').then(() => showToast("Measurement Stopped"));
document.getElementById('btn-learn').onclick = () => { 
    isLearning = !isLearning; 
    callAPI(`/noise_learn?active=${isLearning}`).then(() => showToast(isLearning ? "Noise Learning ON" : "Noise Learning OFF")); 
};

document.getElementById('btn-calibrate').onclick = async () => {
    if(!confirm("Z-축이 위를 향하도록 수평 상태를 유지하십시오. 계속하시겠습니까?")) return;
    showToast("Calibrating...");
    try {
        const res = await callAPI('/calibrate');
        if ((await res.json()).ok) showToast("Calibration Success!");
        else showToast("Calibration Failed", true);
    } catch(e) { showToast("Network Error", true); }
};

async function rebootDevice() {
    if (!confirm("장치를 재부팅하시겠습니까?")) return;
    try {
        await callAPI('/reboot');
        showToast("Rebooting... Please wait.");
        setTimeout(() => location.reload(), T20_CONST.UI.REBOOT_RELOAD_MS);
    } catch(e) { showToast("Reboot command sent.", true); setTimeout(() => location.reload(), T20_CONST.UI.REBOOT_RELOAD_MS); }
}

async function uploadOTA() {
    const fileInput = document.getElementById('ota-file');
    if (!fileInput.files.length) return showToast("Please select a .bin file", true);
    if (!confirm("펌웨어 업데이트 중에는 전원을 끄지 마십시오. 진행하시겠습니까?")) return;

    const formData = new FormData();
    formData.append("update", fileInput.files[0]);

    const progressText = document.getElementById('ota-progress');
    progressText.style.display = 'block';
    
    const xhr = new XMLHttpRequest();
    xhr.open("POST", `${T20_CONST.API_BASE}/ota_update`, true);
    
    xhr.upload.onprogress = (e) => {
        if (e.lengthComputable) {
            const percent = Math.round((e.loaded / e.total) * 100);
            progressText.innerText = `Uploading & Flashing: ${percent}%`;
        }
    };
    
    xhr.onload = () => {
        if (xhr.status === 200) {
            progressText.innerText = "Success! Rebooting...";
            showToast("OTA Success! Device is rebooting.");
            setTimeout(() => location.reload(), T20_CONST.UI.REBOOT_RELOAD_MS + 2000);
        } else {
            progressText.style.display = 'none';
            showToast("OTA Failed.", true);
        }
    };
    xhr.send(formData);
}

// [기능 유지] Config 백업 및 복원 로직
async function downloadConfig() {
    try {
        const res = await fetch(`${T20_CONST.API_BASE}/runtime_config`);
        if (!res.ok) throw new Error("Failed to fetch config");
        const cfg = await res.json();
        
        const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(cfg, null, 2));
        const downloadAnchorNode = document.createElement('a');
        downloadAnchorNode.setAttribute("href", dataStr);
        downloadAnchorNode.setAttribute("download", "t20_config_backup.json");
        document.body.appendChild(downloadAnchorNode);
        downloadAnchorNode.click();
        downloadAnchorNode.remove();
        
        showToast("Config downloaded successfully!");
    } catch (e) {
        showToast("Config download failed", true);
    }
}

function uploadConfig(event) {
    const file = event.target.files[0];
    if (!file) return;
    
    const reader = new FileReader();
    reader.onload = async function(e) {
        try {
            const jsonStr = e.target.result;
            JSON.parse(jsonStr); // 유효성 1차 검사
            
            if (!confirm("선택한 설정 파일로 덮어쓰고 기기를 재부팅하시겠습니까?")) {
                event.target.value = ""; 
                return;
            }
            
            showToast("Uploading config...");
            
            const res = await fetch(`${T20_CONST.API_BASE}/runtime_config`, {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: jsonStr
            });
            
            if (res.ok) {
                showToast("Config restored! Rebooting...");
                setTimeout(() => location.reload(), T20_CONST.UI.REBOOT_RELOAD_MS);
            } else {
                showToast("Upload failed by device", true);
            }
        } catch (err) {
            showToast("Invalid JSON file format", true);
        }
        event.target.value = ""; 
    };
    
    reader.readAsText(file);
}

// ==========================================
// 5. 설정 데이터 동기화 
// ==========================================
const getNested = (obj, path) => path.split('.').reduce((acc, part) => acc && acc[part] !== undefined ? acc[part] : undefined, obj);
const setNested = (obj, path, value) => {
    const keys = path.split('.');
    let current = obj;
    for (let i = 0; i < keys.length - 1; i++) {
        if (!current[keys[i]]) current[keys[i]] = {};
        current = current[keys[i]];
    }
    current[keys[keys.length - 1]] = value;
};

async function loadSettings() {
    try {
        const res = await fetch(`${T20_CONST.API_BASE}/runtime_config`);
        const cfg = await res.json();
        
        sysState.fft_size = parseInt(cfg.feature?.fft_size || 256);
        sysState.axis_count = parseInt(cfg.feature?.axis_count || 1);
        sysState.mfcc_coeffs = parseInt(cfg.feature?.mfcc_coeffs || 13);
        sysState.sequence_frames = parseInt(cfg.output?.sequence_frames || 16);
        updateChartDimensions();

        ['form-dsp', 'form-output', 'form-sysadmin'].forEach(formId => {
            const form = document.getElementById(formId);
            if(!form) return;
            form.querySelectorAll('select, input').forEach(el => {
                if (el.name.startsWith('multi_') || el.name.startsWith('band_')) return;
                const val = getNested(cfg, el.name);
                if (val !== undefined) el.value = typeof val === 'boolean' ? val.toString() : val;
            });
        });

        if(cfg.trigger && cfg.trigger.sw_event && cfg.trigger.sw_event.bands) {
            const f_dsp = document.getElementById('form-dsp');
            cfg.trigger.sw_event.bands.forEach((b, i) => {
                if(i > 2) return;
                f_dsp[`band_${i}_enable`].value = b.enable ? "true" : "false";
                f_dsp[`band_${i}_start`].value = b.start_hz || "";
                f_dsp[`band_${i}_end`].value = b.end_hz || "";
                f_dsp[`band_${i}_thresh`].value = b.threshold || "";
            });
        }

        if (cfg.wifi && cfg.wifi.multi_ap && Array.isArray(cfg.wifi.multi_ap)) {
            const f_adm = document.getElementById('form-sysadmin');
            cfg.wifi.multi_ap.forEach((ap, i) => {
                if (i >= T20_CONST.UI.MAX_ROUTERS) return;
                f_adm[`multi_ssid_${i}`].value = ap.ssid || "";
                f_adm[`multi_pass_${i}`].value = ap.password || "";
                f_adm[`multi_static_${i}`].value = ap.use_static_ip ? "true" : "false";
                f_adm[`multi_ip_${i}`].value = ap.local_ip || "";
                f_adm[`multi_gw_${i}`].value = ap.gateway || "";
                f_adm[`multi_sn_${i}`].value = ap.subnet || "";
                f_adm[`multi_dns1_${i}`].value = ap.dns1 || "";
                f_adm[`multi_dns2_${i}`].value = ap.dns2 || "";
                document.getElementById(`rip_${i}`).style.display = ap.use_static_ip ? 'grid' : 'none';
            });
        }
        toggleWifiFields();
    } catch(e) { console.warn("Config load fail", e); }
}

async function saveSettings() {
    const configData = {};
    
    ['form-dsp', 'form-output', 'form-sysadmin'].forEach(formId => {
        const form = document.getElementById(formId);
        if(!form) return;
        new FormData(form).forEach((value, key) => {
            if (key.startsWith('multi_') || key.startsWith('band_')) return;
            let parsed = (value === 'true') ? true : (value === 'false') ? false : (!isNaN(value) && value.trim() !== '') ? Number(value) : value;
            setNested(configData, key, parsed);
        });
    });

    const f_dsp = document.getElementById('form-dsp');
    const bands = [];
    for(let i=0; i<3; i++) {
        bands.push({
            enable: f_dsp[`band_${i}_enable`].value === 'true',
            start_hz: Number(f_dsp[`band_${i}_start`].value),
            end_hz: Number(f_dsp[`band_${i}_end`].value),
            threshold: Number(f_dsp[`band_${i}_thresh`].value)
        });
    }
    
    if(!configData.trigger) configData.trigger = {};
    if(!configData.trigger.sw_event) configData.trigger.sw_event = {};
    configData.trigger.sw_event.bands = bands;

    const multiApArray = [];
    const f_adm = document.getElementById('form-sysadmin');
    for(let i=0; i < T20_CONST.UI.MAX_ROUTERS; i++) {
        const ssid = f_adm.querySelector(`[name="multi_ssid_${i}"]`).value;
        if (ssid.trim()) {
            multiApArray.push({
                ssid: ssid, 
                password: f_adm.querySelector(`[name="multi_pass_${i}"]`).value,
                use_static_ip: f_adm.querySelector(`[name="multi_static_${i}"]`).value === 'true',
                local_ip: f_adm.querySelector(`[name="multi_ip_${i}"]`).value,
                gateway: f_adm.querySelector(`[name="multi_gw_${i}"]`).value,
                subnet: f_adm.querySelector(`[name="multi_sn_${i}"]`).value,
                dns1: f_adm.querySelector(`[name="multi_dns1_${i}"]`).value,
                dns2: f_adm.querySelector(`[name="multi_dns2_${i}"]`).value
            });
        }
    }
    if(!configData.wifi) configData.wifi = {};
    configData.wifi.multi_ap = multiApArray;

    try {
        const res = await fetch(`${T20_CONST.API_BASE}/runtime_config`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(configData)
        });
        if(res.ok) showToast("Settings saved! Rebooting...");
        else showToast("Save failed", true);
    } catch (e) { showToast("Network error", true); }
}

// ==========================================
// 6. 파일 관리 및 진단 로직 
// ==========================================
async function refreshFileList() {
    const list = document.getElementById('file-list');
    list.innerHTML = '<tr><td colspan="4">Loading...</td></tr>';
    try {
        const res = await fetch(`${T20_CONST.API_BASE}/recorder_index`);
        const data = await res.json();
        list.innerHTML = data.items.map(f => `
            <tr>
                <td>${f.path.split('/').pop()}</td>
                <td>${(f.size_bytes/1024).toFixed(1)} KB</td>
                <td>${f.record_count}</td>
                <td><button class="btn primary" onclick="window.location.href='${T20_CONST.API_BASE}/recorder/download?path=${f.path}'">Down</button></td>
            </tr>`).join('');
    } catch(e) { list.innerHTML = '<tr><td colspan="4">No record index found.</td></tr>'; }
}

async function fetchDiagnostics() {
    try {
        const res = await fetch(`${T20_CONST.API_BASE}/status`);
        const d = await res.json();
        
        document.getElementById('val-dropped').innerText = "0";
        document.getElementById('val-samples').innerText = d.hash || "0";
        document.getElementById('diag-health').innerText = JSON.stringify(d, null, 2);
        document.getElementById('diag-sensor').innerText = d.sensor_status || "Active";

        const tag = document.getElementById('status-tag');
        if (d.running === false) {
            tag.className = "tag offline"; 
            tag.innerText = "HALTED";
        } else if (d.sys_state !== undefined) {
            switch(d.sys_state) {
                case 0: tag.className = "tag offline"; tag.innerText = "INIT"; break;
                case 1: tag.className = "tag ready"; tag.innerText = "READY"; break;
                case 2: tag.className = "tag monitoring"; tag.innerText = "MONITORING"; break;
                case 3: tag.className = "tag recording"; tag.innerText = "RECORDING"; break;
                case 4: tag.className = "tag learning"; tag.innerText = "LEARNING"; break;
                case 5: tag.className = "tag error"; tag.innerText = "ERROR"; break;
                default: tag.className = "tag online"; tag.innerText = "UNKNOWN"; break;
            }
        }
    } catch(e) { 
        document.getElementById('diag-health').innerText = "API Error"; 
        const tag = document.getElementById('status-tag');
        tag.className = "tag offline"; 
        tag.innerText = "API OFFLINE";
    }
}

window.onload = () => {
    initUI();
    initCharts();
    connectWebSocket();
    loadSettings();
    refreshFileList();
    
    requestAnimationFrame(renderLoop);
};

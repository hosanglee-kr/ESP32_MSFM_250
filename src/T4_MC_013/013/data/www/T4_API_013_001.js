// T4_API_013_01.js
const API_BASE = "/api";
const WS_CONST = { PROTOCOL: window.location.protocol === 'https:' ? 'wss:' : 'ws:', URI: '/ws', RECONNECT_MS: 2000 };
const HISTORY_LEN = 300; 

window.dashState = {
    rms: 0, sta_lta: 0, centroid: 0,
    peaks: [{f:0,a:0},{f:0,a:0},{f:0,a:0},{f:0,a:0},{f:0,a:0}],
    thresh: { rms: 0.05, stalta: 3.0 },
    
    wave: [Array.from({length: 1024}, (_, i) => i), new Float32Array(1024)],
    spec: [Array.from({length: 513}, (_, i) => i * (42000/1024)), new Float32Array(513)],
    
    // [v013] 캘리브레이션 렌더링용 배열 추가
    calibGain: [Array.from({length: 513}, (_, i) => i * (42000/1024)), new Float32Array(513)],
    calibFir: [Array.from({length: 63}, (_, i) => i - 31), new Float32Array(63)],
    
    telemetry: [ [], [], [] ], 
    mfccLine: [ [], [], [] ],  
    mfccBar: [ Array.from({length: 39}, (_, i) => i), new Float32Array(39) ],
    isDirty: false
};

const SMEA_API = {
    postCommand: async (path) => fetch(`${API_BASE}${path}`, { method: 'POST' }),
    fetchDiagnostics: async () => { const res = await fetch(`${API_BASE}/status`); if (!res.ok) throw new Error("API Offline"); return await res.json(); },
    loadRuntimeConfig: async () => { const res = await fetch(`${API_BASE}/runtime_config`); if (!res.ok) throw new Error("Config not found"); return await res.json(); },
    
    // [v013 신규 API] 핫스왑 제어 및 스로틀링
    previewConfig: async (configObj) => { const res = await fetch(`${API_BASE}/config/preview`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(configObj) }); return res.ok; },
    commitConfig: async () => { const res = await fetch(`${API_BASE}/config/save`, { method: 'POST' }); return res.ok; },
    cancelConfig: async () => { const res = await fetch(`${API_BASE}/config/cancel`, { method: 'POST' }); return res.ok; },
    setStreamControl: async (obj) => { const res = await fetch(`${API_BASE}/stream/control`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(obj) }); return res.ok; },

    // [기존 API 유지]
    saveRuntimeConfig: async (configObj) => { const res = await fetch(`${API_BASE}/runtime_config`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(configObj) }); return res.ok; },
    downloadBackup: async () => { const res = await fetch(`${API_BASE}/runtime_config`); const cfg = await res.json(); const a = document.createElement('a'); a.href = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(cfg, null, 2)); a.download = "smea100_backup.json"; a.click(); },
    uploadOTA: (file, onProgress, onSuccess, onFail) => { const formData = new FormData(); formData.append("update", file); const xhr = new XMLHttpRequest(); xhr.open("POST", `${API_BASE}/ota`, true); xhr.upload.onprogress = onProgress; xhr.onload = () => { if (xhr.status === 200) onSuccess(); else onFail(); }; xhr.onerror = onFail; xhr.send(formData); }
};

let ws = null;
let isMockMode = false;
let mockInterval = null;

function connectWebSocket() {
    if (isMockMode) return;
    ws = new WebSocket(`${WS_CONST.PROTOCOL}//${window.location.host}${WS_CONST.URI}`);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => console.log("[WS] Connected");
    ws.onclose = () => { console.warn("[WS] Disconnected."); if(!isMockMode) setTimeout(connectWebSocket, WS_CONST.RECONNECT_MS); };
    
    ws.onmessage = (event) => {
        if (isMockMode || !(event.data instanceof ArrayBuffer)) return;
        const dv = new DataView(event.data);
        if (dv.byteLength < 4 || dv.getUint8(0) !== 0xA5) return; 
        
        const type = dv.getUint8(1);
        const tNow = Date.now() / 1000;

        if (type === 0x01) { 
            window.dashState.rms = dv.getFloat32(8, true);
            window.dashState.sta_lta = dv.getFloat32(12, true);
            window.dashState.centroid = dv.getFloat32(20, true);
            for (let i = 0; i < 5; i++) {
                window.dashState.peaks[i].f = dv.getFloat32(56 + (i * 4), true);
                window.dashState.peaks[i].a = dv.getFloat32(76 + (i * 4), true);
            }
            for (let i = 0; i < 39; i++) window.dashState.mfccBar[1][i] = dv.getFloat32(96 + (i * 4), true);

            pushHistory(window.dashState.telemetry, tNow, [window.dashState.rms, window.dashState.sta_lta]);
            pushHistory(window.dashState.mfccLine, tNow, [window.dashState.mfccBar[1][0], window.dashState.mfccBar[1][1]]);
            window.dashState.isDirty = true;
        } 
        else if (type === 0x02) { 
            for (let i = 0; i < 513; i++) window.dashState.spec[1][i] = dv.getFloat32(4 + (i * 4), true);
            window.dashState.isDirty = true;
        }
        else if (type === 0x03) { 
            for (let i = 0; i < 1024; i++) window.dashState.wave[1][i] = dv.getFloat32(4 + (i * 4), true);
            window.dashState.isDirty = true;
        }
        // [v013 파서 추가] 캘리브레이션 
        else if (type === 0x04) {
            for (let i = 0; i < 513; i++) window.dashState.calibGain[1][i] = dv.getFloat32(4 + (i * 4), true);
            for (let i = 0; i < 63; i++) window.dashState.calibFir[1][i] = dv.getFloat32(4 + (513 * 4) + (i * 4), true);
            window.dashState.isDirty = true;
        }
    };
}

function pushHistory(histArr, t, values) {
    histArr[0].push(t);
    for(let i=0; i<values.length; i++) histArr[i+1].push(values[i]);
    if (histArr[0].length > HISTORY_LEN) {
        histArr[0].shift();
        for(let i=0; i<values.length; i++) histArr[i+1].shift();
    }
}

function toggleMockMode(enabled) {
    isMockMode = enabled;
    if (isMockMode) {
        if (ws) { ws.close(); ws = null; }
        if (mockInterval) clearInterval(mockInterval);
        
        let tIdx = 0;
        mockInterval = setInterval(() => {
            const t = Date.now() / 1000;
            const isImpulse = Math.random() > 0.95; 
            
            window.dashState.rms = Math.sin(t * 3) * 0.5 + 0.5 + (isImpulse ? 2.0 : 0);
            window.dashState.sta_lta = isImpulse ? 5.0 : Math.random() * 0.5 + 1.0;
            window.dashState.centroid = 1000 + Math.sin(t) * 500;
            
            for(let i=0; i<5; i++) { 
                window.dashState.peaks[i].f = 120 * (i+1) + (Math.random()*10); 
                window.dashState.peaks[i].a = Math.random() * (isImpulse ? 5 : 1); 
            }
            for(let i=0; i<39; i++) { 
                const scale = i < 13 ? 15 : (i < 26 ? 5 : 2);
                window.dashState.mfccBar[1][i] = Math.sin(t * (i+1) * 0.5) * scale + (Math.random()*scale*0.2); 
            }
            
            pushHistory(window.dashState.telemetry, t, [window.dashState.rms, window.dashState.sta_lta]);
            pushHistory(window.dashState.mfccLine, t, [window.dashState.mfccBar[1][0], window.dashState.mfccBar[1][1]]);
            
            if (tIdx++ % 10 === 0) {
                for (let i = 0; i < 1024; i++) window.dashState.wave[1][i] = Math.sin(i * 0.1 + t) * (isImpulse ? 2 : 0.5) + (Math.random() * 0.1 - 0.05);
                for (let i = 0; i < 513; i++) window.dashState.spec[1][i] = Math.abs(Math.sin(i * 0.05)) * (isImpulse ? 10 : 2) + Math.random();
            }
            window.dashState.isDirty = true;
        }, 10); 
        
        document.getElementById('status-tag').className = "tag ready"; 
        document.getElementById('status-tag').innerText = "SIMULATION";
    } else {
        if (mockInterval) { clearInterval(mockInterval); mockInterval = null; }
        connectWebSocket();
    }
}

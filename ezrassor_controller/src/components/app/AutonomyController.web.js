/**
 * AutonomyController.web.js
 * ──────────────────────────
 * Autonomy control screen for the RE-RASSOR rover.
 *
 * Features:
 *  • 11×11 navigation grid (−5 m to +5 m in X and Y, 1 m / cell).
 *    Tap a cell to select a goal; press NAVIGATE to send it via POST /navigate.
 *  • Live depth camera feed rendered on an HTML canvas from the WebSocket
 *    `depth_image` event (grayscale, ~10 Hz).
 *  • Real-time odometry display from the WebSocket `odom` event.
 *  • CALIBRATE button → POST /calibrate.
 *  • Robot position (R) and active nav goal (G) are highlighted in the grid.
 *
 * Backend events used:
 *   subscribe_odom         → `odom`         {x, y, yaw}
 *   subscribe_depth_image  → `depth_image`  {data: base64, width, height}
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { io } from 'socket.io-client';
import { useFonts } from 'expo-font';

// ── Grid constants ─────────────────────────────────────────────────────────────
const GRID_COLS  = 11;   // columns (X axis)
const GRID_ROWS  = 11;   // rows    (Y axis, top = positive)
const GRID_MIN_X = -5;   // metres
const GRID_MAX_X =  5;
const GRID_MIN_Y = -5;
const GRID_MAX_Y =  5;

function colToWorldX(col) { return GRID_MIN_X + col; }
function rowToWorldY(row) { return GRID_MAX_Y - row; }
function worldXToCol(x)   { return Math.round(x - GRID_MIN_X); }
function worldYToRow(y)   { return Math.round(GRID_MAX_Y - y); }

// ── Costmap helpers ────────────────────────────────────────────────────────────
// Returns a background color for a grid cell based on the Nav2 costmap value
// at world position (wx, wy). Returns null if no costmap is available.
function getCostmapBg(costmap, wx, wy) {
    if (!costmap) return null;
    const gx = Math.floor((wx - costmap.origin_x) / costmap.resolution);
    const gy = Math.floor((wy - costmap.origin_y) / costmap.resolution);
    if (gx < 0 || gx >= costmap.width || gy < 0 || gy >= costmap.height) return '#1a2020'; // out-of-bounds = unknown
    const v = costmap.data[gy * costmap.width + gx];
    if (v < 0)    return '#1a2020';  // unknown
    if (v === 0)  return '#2a2d2d';  // free
    if (v < 50)   return '#3a2e08';  // low inflation (dark amber)
    if (v < 100)  return '#4a2008';  // high inflation (dark orange)
    return '#5a1010';                // lethal obstacle (dark red)
}

// ── Component ──────────────────────────────────────────────────────────────────
export default function AutonomyController({ navigation }) {
    const [fontsLoaded] = useFonts({ NasaFont: require('../../../assets/nasa.ttf') });

    const [ip,        setIp]        = useState('');
    const [connected, setConnected] = useState(false);
    const [status,    setStatus]    = useState('Connecting…');
    const [odom,      setOdom]      = useState({ x: 0, y: 0, yaw: 0 });
    const [selected,  setSelected]  = useState(null); // {col, row, wx, wy}
    const [navGoal,   setNavGoal]   = useState(null); // {x, y} — last confirmed goal
    const [depthSize, setDepthSize] = useState({ w: 640, h: 480 });
    const [hasDepth,  setHasDepth]  = useState(false);
    const [costmap,   setCostmap]   = useState(null); // {width, height, resolution, origin_x, origin_y, data[]}

    const socketRef        = useRef(null);
    const canvasRef        = useRef(null);
    const costmapCanvasRef = useRef(null);

    // ── Load IP from AsyncStorage then connect ─────────────────────────────────
    useEffect(() => {
        let socket = null;

        AsyncStorage.getItem('myIp').then((storedIp) => {
            const host = storedIp || '192.168.1.2:5000';
            setIp(host);

            const url = host.startsWith('http') ? host : `http://${host}`;
            socket = io(url, {
                transports:        ['websocket'],
                reconnection:      true,
                reconnectionDelay: 2000,
                timeout:           5000,
            });
            socketRef.current = socket;

            socket.on('connect', () => {
                setConnected(true);
                setStatus('Connected');
                socket.emit('subscribe_odom');
                socket.emit('subscribe_depth_image');
                socket.emit('subscribe_costmap');
            });

            socket.on('disconnect', () => {
                setConnected(false);
                setStatus('Disconnected — retrying…');
            });

            socket.on('connect_error', (err) => {
                setConnected(false);
                setStatus(`Cannot reach ${host}: ${err.message}`);
            });

            socket.on('odom', (data) => {
                setOdom({ x: data.x ?? 0, y: data.y ?? 0, yaw: data.yaw ?? 0 });
            });

            socket.on('calibrated', () => {
                setOdom({ x: 0, y: 0, yaw: 0 });
                setNavGoal(null);
                setSelected(null);
                setStatus('Calibrated — position reset to (0, 0)');
            });

            socket.on('depth_image', (data) => {
                renderDepth(data);
            });

            socket.on('costmap', (data) => {
                setCostmap(data);
            });
        });

        return () => {
            if (socket) {
                socket.emit('unsubscribe_odom');
                socket.emit('unsubscribe_depth_image');
                socket.emit('unsubscribe_costmap');
                socket.disconnect();
            }
        };
    }, []); // run once on mount

    // ── Render depth image onto the canvas ─────────────────────────────────────
    const renderDepth = useCallback((data) => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const { data: b64, width: W, height: H } = data;
        if (!b64 || !W || !H) return;

        const binary = atob(b64);
        if (binary.length !== W * H) return; // wrong size — drop frame

        setDepthSize({ w: W, h: H });
        setHasDepth(true);
        canvas.width  = W;
        canvas.height = H;

        const ctx = canvas.getContext('2d');
        const imgData = ctx.createImageData(W, H);
        for (let i = 0; i < W * H; i++) {
            const v = binary.charCodeAt(i);
            imgData.data[i * 4]     = v;
            imgData.data[i * 4 + 1] = v;
            imgData.data[i * 4 + 2] = v;
            imgData.data[i * 4 + 3] = 255;
        }
        ctx.putImageData(imgData, 0, 0);
    }, []);

    // ── Render costmap onto the dedicated canvas ───────────────────────────────
    useEffect(() => {
        const canvas = costmapCanvasRef.current;
        if (!canvas || !costmap) return;

        const { width: W, height: H, data } = costmap;
        if (!W || !H) return;
        canvas.width  = W;
        canvas.height = H;

        const ctx     = canvas.getContext('2d');
        const imgData = ctx.createImageData(W, H);

        for (let i = 0; i < W * H; i++) {
            const v = data[i];
            let r, g, b;
            if      (v < 0)   { r = 26;  g = 32;  b = 32;  }   // unknown
            else if (v === 0) { r = 42;  g = 45;  b = 45;  }   // free
            else if (v < 50)  { r = 58;  g = 46;  b = 8;   }   // low inflation
            else if (v < 100) { r = 74;  g = 32;  b = 8;   }   // high inflation
            else              { r = 90;  g = 16;  b = 16;  }   // lethal obstacle
            imgData.data[i * 4]     = r;
            imgData.data[i * 4 + 1] = g;
            imgData.data[i * 4 + 2] = b;
            imgData.data[i * 4 + 3] = 255;
        }
        ctx.putImageData(imgData, 0, 0);

        // Draw robot position (green dot)
        const rx = (odom.x - costmap.origin_x) / costmap.resolution;
        const ry = H - (odom.y - costmap.origin_y) / costmap.resolution;
        ctx.beginPath();
        ctx.arc(rx, ry, Math.max(3, 4 / costmap.resolution / 20), 0, 2 * Math.PI);
        ctx.fillStyle = '#00c853';
        ctx.fill();

        // Draw nav goal (orange dot)
        if (navGoal) {
            const gx = (navGoal.x - costmap.origin_x) / costmap.resolution;
            const gy = H - (navGoal.y - costmap.origin_y) / costmap.resolution;
            ctx.beginPath();
            ctx.arc(gx, gy, Math.max(3, 4 / costmap.resolution / 20), 0, 2 * Math.PI);
            ctx.fillStyle = '#e65100';
            ctx.fill();
        }
    }, [costmap, odom.x, odom.y, navGoal]);

    // ── Navigation grid cell press ─────────────────────────────────────────────
    const handleCellPress = (col, row) => {
        setSelected({ col, row, wx: colToWorldX(col), wy: rowToWorldY(row) });
    };

    // ── Send navigation goal to backend ───────────────────────────────────────
    const handleNavigate = async () => {
        if (!selected) return;
        try {
            const url = ip.startsWith('http') ? ip : `http://${ip}`;
            const res = await fetch(`${url}/navigate`, {
                method:  'POST',
                headers: { 'Content-Type': 'application/json' },
                body:    JSON.stringify({ x: selected.wx, y: selected.wy, theta: 0 }),
            });
            if (res.ok) {
                setNavGoal({ x: selected.wx, y: selected.wy });
                setStatus(`Navigating → (${selected.wx} m, ${selected.wy} m)`);
            } else {
                setStatus(`/navigate returned ${res.status}`);
            }
        } catch (e) {
            setStatus(`Navigate error: ${e.message}`);
        }
    };

    // ── Force stop — send zero wheel command ──────────────────────────────────
    const handleForceStop = async () => {
        try {
            const url = ip.startsWith('http') ? ip : `http://${ip}`;
            await fetch(`${url}/stop`, {
                method:  'POST',
                headers: { 'Content-Type': 'application/json' },
            });
            setNavGoal(null);
            setSelected(null);
            setStatus('FORCE STOP sent — rover halted');
        } catch (e) {
            setStatus(`Force stop error: ${e.message}`);
        }
    };

    // ── Send calibrate request ─────────────────────────────────────────────────
    const handleCalibrate = async () => {
        try {
            const url = ip.startsWith('http') ? ip : `http://${ip}`;
            setStatus('Calibrating…');
            const res = await fetch(`${url}/calibrate`, { method: 'POST' });
            if (res.ok) {
                setOdom({ x: 0, y: 0, yaw: 0 });
                setNavGoal(null);
                setSelected(null);
                setStatus('Calibrated — position reset to (0, 0)');
            } else {
                setStatus(`/calibrate returned ${res.status}`);
            }
        } catch (e) {
            setStatus(`Calibrate error: ${e.message}`);
        }
    };

    // ── Derive robot grid position from odometry ───────────────────────────────
    const robotCol = worldXToCol(odom.x);
    const robotRow = worldYToRow(odom.y);
    const robotInGrid =
        robotCol >= 0 && robotCol < GRID_COLS &&
        robotRow >= 0 && robotRow < GRID_ROWS;

    const goalCol = navGoal ? worldXToCol(navGoal.x) : -1;
    const goalRow = navGoal ? worldYToRow(navGoal.y) : -1;

    const yawDeg = ((odom.yaw * 180) / Math.PI).toFixed(1);

    // ── Render ─────────────────────────────────────────────────────────────────
    if (!fontsLoaded) return <View style={styles.root} />;

    return (
        <View style={styles.root}>
            {/* ── Header ────────────────────────────────────────────────────── */}
            <View style={styles.header}>
                <View style={styles.headerLeft}>
                    <Pressable style={styles.backBtn} onPress={() => navigation.goBack()}>
                        <Text style={styles.backBtnText}>← BACK</Text>
                    </Pressable>
                    <Pressable style={styles.sshBtn} onPress={() => navigation.navigate('SSH Terminal Screen')}>
                        <Text style={styles.sshBtnText}>SSH TERMINAL</Text>
                    </Pressable>
                </View>
                <Text style={styles.headerTitle}>AUTONOMY CONTROL</Text>
                <View style={styles.headerRight}>
                    <View style={[styles.dot, { backgroundColor: connected ? '#00c853' : '#f44336' }]} />
                    <Text style={styles.headerIp}>{ip || '—'}</Text>
                </View>
            </View>

            {/* ── Status bar ────────────────────────────────────────────────── */}
            <View style={styles.statusBar}>
                <Text style={styles.statusText}>{status}</Text>
            </View>

            {/* ── Scrollable content area ───────────────────────────────────── */}
            <ScrollView style={styles.scrollArea} contentContainerStyle={styles.scrollContent}>

            {/* ── Body ──────────────────────────────────────────────────────── */}
            <View style={styles.body}>

                {/* ── LEFT: Navigation grid ─────────────────────────────────── */}
                <View style={styles.leftPanel}>
                    <Text style={styles.sectionTitle}>NAVIGATION MAP</Text>
                    <Text style={styles.gridHint}>
                        {selected
                            ? `Target: (${selected.wx} m, ${selected.wy} m)`
                            : 'Tap a cell to select a navigation target'}
                    </Text>

                    <View style={styles.gridOuter}>
                        {/* Y-axis labels */}
                        <View style={styles.yLabels}>
                            {Array.from({ length: GRID_ROWS }, (_, row) => (
                                <Text key={row} style={styles.axisLabel}>
                                    {rowToWorldY(row)}
                                </Text>
                            ))}
                        </View>

                        {/* Grid */}
                        <View style={styles.grid}>
                            {Array.from({ length: GRID_ROWS }, (_, row) => (
                                <View key={row} style={styles.gridRow}>
                                    {Array.from({ length: GRID_COLS }, (_, col) => {
                                        const isRobot    = robotInGrid && col === robotCol && row === robotRow;
                                        const isSelected = selected && col === selected.col && row === selected.row;
                                        const isGoal     = col === goalCol && row === goalRow;
                                        const isOrigin   = col === worldXToCol(0) && row === worldYToRow(0);

                                        let bg = '#2a2d2d';
                                        if (isOrigin && !isRobot && !isSelected && !isGoal) bg = '#1e2a2a';
                                        if (isGoal)     bg = '#e65100';   // orange — nav goal
                                        if (isSelected) bg = '#1565c0';   // blue   — selected
                                        if (isRobot)    bg = '#00c853';   // green  — robot

                                        return (
                                            <Pressable
                                                key={col}
                                                style={[styles.cell, { backgroundColor: bg }]}
                                                onPress={() => handleCellPress(col, row)}
                                            >
                                                {isRobot && (
                                                    <Text style={styles.cellLabel}>R</Text>
                                                )}
                                                {isGoal && !isRobot && (
                                                    <Text style={styles.cellLabel}>G</Text>
                                                )}
                                            </Pressable>
                                        );
                                    })}
                                </View>
                            ))}

                            {/* X-axis labels */}
                            <View style={styles.gridRow}>
                                {Array.from({ length: GRID_COLS }, (_, col) => (
                                    <Text key={col} style={styles.xAxisLabel}>
                                        {colToWorldX(col)}
                                    </Text>
                                ))}
                            </View>
                        </View>
                    </View>

                    {/* Legend */}
                    <View style={styles.legend}>
                        <View style={[styles.legendDot, { backgroundColor: '#00c853' }]} />
                        <Text style={styles.legendText}>Robot</Text>
                        <View style={[styles.legendDot, { backgroundColor: '#1565c0' }]} />
                        <Text style={styles.legendText}>Selected</Text>
                        <View style={[styles.legendDot, { backgroundColor: '#e65100' }]} />
                        <Text style={styles.legendText}>Nav goal</Text>
                    </View>

                    {/* Navigate button */}
                    <Pressable
                        style={[styles.navBtn, !selected && styles.navBtnDisabled]}
                        onPress={handleNavigate}
                        disabled={!selected}
                    >
                        <Text style={styles.navBtnText}>
                            {selected
                                ? `NAVIGATE → (${selected.wx}, ${selected.wy})`
                                : 'SELECT A CELL FIRST'}
                        </Text>
                    </Pressable>
                </View>

                {/* ── RIGHT: Depth feed + info + controls ───────────────────── */}
                <View style={styles.rightPanel}>
                    <Text style={styles.sectionTitle}>DEPTH CAMERA</Text>

                    {/* Canvas — react-native-web passes <canvas> through to DOM */}
                    <View style={styles.canvasWrapper}>
                        <canvas
                            ref={canvasRef}
                            width={640}
                            height={480}
                            style={{
                                width:          '100%',
                                height:         '100%',
                                imageRendering: 'pixelated',
                                display:        'block',
                                background:     '#111',
                            }}
                        />
                        {(!connected || (connected && !hasDepth)) && (
                            <View style={styles.canvasOverlay}>
                                <Text style={styles.canvasOverlayText}>
                                    {!connected ? 'Waiting for connection…' : 'No depth signal'}
                                </Text>
                            </View>
                        )}
                    </View>

                    {/* Odometry readout */}
                    <View style={styles.odomBox}>
                        <Text style={styles.odomTitle}>ODOMETRY</Text>
                        <Text style={styles.odomLine}>
                            X: <Text style={styles.odomVal}>{odom.x.toFixed(3)} m</Text>
                            {'   '}
                            Y: <Text style={styles.odomVal}>{odom.y.toFixed(3)} m</Text>
                        </Text>
                        <Text style={styles.odomLine}>
                            Yaw: <Text style={styles.odomVal}>{yawDeg}°</Text>
                        </Text>
                        {navGoal && (
                            <Text style={styles.odomGoal}>
                                Active goal: ({navGoal.x} m, {navGoal.y} m)
                            </Text>
                        )}
                    </View>

                    {/* Control buttons */}
                    <View style={styles.ctrlRow}>
                        <Pressable style={styles.forceStopBtn} onPress={handleForceStop}>
                            <Text style={styles.ctrlBtnText}>⬛ FORCE STOP</Text>
                        </Pressable>
                        <Pressable style={styles.calibrateBtn} onPress={handleCalibrate}>
                            <Text style={styles.ctrlBtnText}>CALIBRATE</Text>
                        </Pressable>
                    </View>
                </View>
            </View>

            {/* ── COSTMAP PANEL ───────────────────────────────────────────── */}
            {/* Sits below the body; ScrollView lets it scroll into view without compressing the body */}
            <View style={styles.costmapPanel}>
                <View style={styles.costmapHeader}>
                    <Text style={styles.sectionTitle}>GLOBAL COSTMAP</Text>
                    {costmap && (
                        <Text style={styles.costmapMeta}>
                            {costmap.width}×{costmap.height} px  ·  {costmap.resolution} m/cell
                        </Text>
                    )}
                </View>
                <View style={styles.costmapCanvasWrapper}>
                    <canvas
                        ref={costmapCanvasRef}
                        width={costmap?.width ?? 400}
                        height={costmap?.height ?? 400}
                        style={{
                            height:         '100%',
                            width:          'auto',
                            imageRendering: 'pixelated',
                            display:        'block',
                            background:     '#111',
                        }}
                    />
                    {!costmap && (
                        <View style={styles.canvasOverlay}>
                            <Text style={styles.canvasOverlayText}>
                                Waiting for Nav2 global costmap…
                            </Text>
                        </View>
                    )}
                </View>
                <View style={styles.legend}>
                    <View style={[styles.legendDot, { backgroundColor: '#1a2020' }]} />
                    <Text style={styles.legendText}>Unknown</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#2a2d2d' }]} />
                    <Text style={styles.legendText}>Free</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#3a2e08' }]} />
                    <Text style={styles.legendText}>Low inflation</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#4a2008' }]} />
                    <Text style={styles.legendText}>High inflation</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#5a1010' }]} />
                    <Text style={styles.legendText}>Obstacle</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#00c853' }]} />
                    <Text style={styles.legendText}>Robot</Text>
                    <View style={[styles.legendDot, { backgroundColor: '#e65100' }]} />
                    <Text style={styles.legendText}>Nav goal</Text>
                </View>
            </View>

            </ScrollView>
        </View>
    );
}

// ── Styles ─────────────────────────────────────────────────────────────────────
const DARK  = '#1a1d1d';
const PANEL = '#232727';
const BORDER = '#383d3d';
const TEXT  = '#e0e0e0';
const DIM   = '#888';
const NASA  = 'NasaFont';

const styles = StyleSheet.create({
    root:          { flex: 1, backgroundColor: DARK, flexDirection: 'column' },

    // Header
    header:        { flexDirection: 'row', alignItems: 'center', backgroundColor: '#111',
                     paddingHorizontal: 16, paddingVertical: 10, borderBottomWidth: 1,
                     borderBottomColor: BORDER },
    headerLeft:    { flexDirection: 'row', alignItems: 'center', gap: 8 },
    backBtn:       { paddingHorizontal: 12, paddingVertical: 6, backgroundColor: '#333',
                     borderRadius: 4 },
    backBtnText:   { color: TEXT, fontSize: 13, fontWeight: '600', fontFamily: NASA },
    sshBtn:        { paddingHorizontal: 12, paddingVertical: 6, backgroundColor: '#1a3a1a',
                     borderRadius: 4, borderWidth: 1, borderColor: '#2e5a2e' },
    sshBtnText:    { color: '#00c853', fontSize: 11, fontWeight: '600',
                     letterSpacing: 1, fontFamily: NASA },
    headerTitle:   { flex: 1, color: '#fff', fontSize: 18, fontWeight: '700',
                     letterSpacing: 2, textAlign: 'center', fontFamily: NASA },
    headerRight:   { flexDirection: 'row', alignItems: 'center', minWidth: 160,
                     justifyContent: 'flex-end' },
    dot:           { width: 8, height: 8, borderRadius: 4, marginRight: 8 },
    headerIp:      { color: DIM, fontSize: 12 },

    // Status bar
    statusBar:     { backgroundColor: '#191c1c', paddingHorizontal: 16, paddingVertical: 4,
                     borderBottomWidth: 1, borderBottomColor: BORDER },
    statusText:    { color: DIM, fontSize: 12 },

    // Scrollable area wrapping body + costmap
    scrollArea:    { flex: 1 },
    scrollContent: { flexGrow: 1, flexDirection: 'column' },

    // Body split — minHeight keeps panels from collapsing when costmap is present
    body:          { flexDirection: 'row', minHeight: 520 },

    // Left panel
    leftPanel:     { flex: 6, backgroundColor: PANEL, padding: 16,
                     borderRightWidth: 1, borderRightColor: BORDER },
    sectionTitle:  { color: '#aaa', fontSize: 11, fontWeight: '700', letterSpacing: 2,
                     marginBottom: 6, fontFamily: NASA },
    gridHint:      { color: DIM, fontSize: 12, marginBottom: 10, minHeight: 18 },

    gridOuter:     { flexDirection: 'row', alignItems: 'flex-start' },
    yLabels:       { marginRight: 4, paddingTop: 0 },
    axisLabel:     { color: DIM, fontSize: 9, height: 36, textAlignVertical: 'center',
                     lineHeight: 36, width: 20, textAlign: 'right' },

    grid:          { flexDirection: 'column' },
    gridRow:       { flexDirection: 'row' },
    cell:          { width: 36, height: 36, margin: 1, borderRadius: 3,
                     alignItems: 'center', justifyContent: 'center',
                     borderWidth: 1, borderColor: '#333' },
    cellLabel:     { color: '#fff', fontSize: 10, fontWeight: '700' },
    xAxisLabel:    { width: 36, textAlign: 'center', color: DIM, fontSize: 9,
                     marginTop: 2, marginHorizontal: 1 },

    legend:        { flexDirection: 'row', alignItems: 'center', marginTop: 12,
                     flexWrap: 'wrap', gap: 8 },
    legendDot:     { width: 10, height: 10, borderRadius: 2, marginRight: 4 },
    legendText:    { color: DIM, fontSize: 11, marginRight: 12 },

    navBtn:        { marginTop: 14, backgroundColor: '#1565c0', borderRadius: 6,
                     paddingVertical: 12, paddingHorizontal: 20, alignItems: 'center' },
    navBtnDisabled:{ backgroundColor: '#333' },
    navBtnText:    { color: '#fff', fontWeight: '700', fontSize: 14, letterSpacing: 1, fontFamily: NASA },

    // Right panel
    rightPanel:    { flex: 4, backgroundColor: '#1e2222', padding: 16,
                     flexDirection: 'column' },
    canvasWrapper: { width: '100%', aspectRatio: 640 / 480, backgroundColor: '#111',
                     borderRadius: 4, overflow: 'hidden', position: 'relative',
                     borderWidth: 1, borderColor: BORDER, marginBottom: 12 },
    canvasOverlay: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0,
                     alignItems: 'center', justifyContent: 'center',
                     backgroundColor: 'rgba(0,0,0,0.6)' },
    canvasOverlayText: { color: DIM, fontSize: 13 },

    odomBox:       { backgroundColor: PANEL, borderRadius: 6, padding: 12,
                     borderWidth: 1, borderColor: BORDER, marginBottom: 12 },
    odomTitle:     { color: '#aaa', fontSize: 10, fontWeight: '700', letterSpacing: 2,
                     marginBottom: 6, fontFamily: NASA },
    odomLine:      { color: TEXT, fontSize: 13, marginBottom: 2 },
    odomVal:       { color: '#4fc3f7', fontWeight: '600' },
    odomGoal:      { color: '#ff8a65', fontSize: 12, marginTop: 4 },

    // Costmap panel
    costmapPanel:        { backgroundColor: PANEL, padding: 16, marginTop: 16,
                           borderTopWidth: 1, borderTopColor: BORDER },
    costmapHeader:       { flexDirection: 'row', alignItems: 'center', marginBottom: 8 },
    costmapMeta:         { color: DIM, fontSize: 11, marginLeft: 12 },
    costmapCanvasWrapper:{ height: 200, backgroundColor: '#111', borderRadius: 4,
                           overflow: 'hidden', position: 'relative', borderWidth: 1,
                           borderColor: BORDER, alignItems: 'center', marginBottom: 8 },

    ctrlRow:       { flexDirection: 'row', gap: 10, flexWrap: 'wrap' },
    forceStopBtn:  { flex: 1, backgroundColor: '#212121', borderRadius: 6,
                     paddingVertical: 12, alignItems: 'center', minWidth: 120,
                     borderWidth: 2, borderColor: '#f44336' },
    calibrateBtn:  { flex: 1, backgroundColor: '#b71c1c', borderRadius: 6,
                     paddingVertical: 12, alignItems: 'center', minWidth: 120 },
    ctrlBtnText:   { color: '#fff', fontWeight: '700', fontSize: 13, letterSpacing: 1, fontFamily: NASA },
});

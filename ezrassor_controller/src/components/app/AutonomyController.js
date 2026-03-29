/**
 * AutonomyController.js — mobile fallback
 * The full autonomy screen (depth canvas + grid map) runs in the web / Electron
 * build via AutonomyController.web.js.  This stub handles the mobile code path.
 */

import React from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';

export default function AutonomyController({ navigation }) {
    return (
        <View style={styles.root}>
            <Text style={styles.title}>AUTONOMY CONTROL</Text>
            <Text style={styles.body}>
                The full autonomy interface (navigation grid + depth camera feed) is
                available in the desktop (Electron) build.{'\n\n'}
                On mobile you can send a navigation goal via the Controller screen
                autonomy modal.
            </Text>
            <Pressable style={styles.btn} onPress={() => navigation.goBack()}>
                <Text style={styles.btnText}>← BACK</Text>
            </Pressable>
        </View>
    );
}

const styles = StyleSheet.create({
    root:    { flex: 1, backgroundColor: '#1a1d1d', alignItems: 'center',
               justifyContent: 'center', padding: 32 },
    title:   { color: '#fff', fontSize: 20, fontWeight: '700', letterSpacing: 2,
               marginBottom: 20 },
    body:    { color: '#aaa', fontSize: 14, textAlign: 'center', lineHeight: 22,
               marginBottom: 32 },
    btn:     { backgroundColor: '#333', borderRadius: 6, paddingVertical: 12,
               paddingHorizontal: 24 },
    btnText: { color: '#fff', fontWeight: '600', fontSize: 14 },
});

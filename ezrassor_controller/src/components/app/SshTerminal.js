// Native stub — SSH terminal only works in the Electron (web) build.
import React from 'react';
import { View, Text, StyleSheet } from 'react-native';

export default function SshTerminal() {
    return (
        <View style={styles.root}>
            <Text style={styles.msg}>SSH terminal is not available on this platform.</Text>
        </View>
    );
}

const styles = StyleSheet.create({
    root: { flex: 1, backgroundColor: '#1a1d1d', alignItems: 'center', justifyContent: 'center' },
    msg:  { color: '#888', fontSize: 14 },
});

import { AnimatePresence, MotiView } from "moti";
import { forwardRef, useImperativeHandle, useState } from "react";
import { StyleSheet, View } from "react-native";

function Shape({ dl }) {
    return (
        <MotiView
            from={{
                opacity: 0,
            }}
            animate={{
                opacity: 1,
            }}
            transition={{
                type: "timing",
                duration: 720,
                loop: true,
                delay: dl,
            }}
            style={[styles.shape, { backgroundColor: "white" }]}
        />
    );
}

export default LoadingDots = forwardRef(({ autoPlay = true }, ref) => {
    const [startAnim, setStartAnim] = useState(autoPlay);

    useImperativeHandle(ref, () => ({
        reset: () => setStartAnim(false),
        play: () => setStartAnim(true),
    }));

    if (!startAnim) {
        return null;
    }

    return (
        <View style={styles.container}>
            <AnimatePresence>
                <Shape dl={0} key="First" />
                <Shape dl={180} key="Second" />
                <Shape dl={270} key="Third" />
            </AnimatePresence>
        </View>
    );
});

const styles = StyleSheet.create({
    shape: {
        justifyContent: "center",
        height: 80,
        width: 80,
        borderRadius: 40,
        marginRight: 10,
        backgroundColor: "white",
    },
    container: {
        flex: 1,
        alignItems: "center",
        justifyContent: "center",
        flexDirection: "row",
        rowGap: 40,
        zIndex: 1000,
    },
});

import { StyleSheet } from 'react-native';

const ControllerStyle = StyleSheet.create({

    buttonLayoutContainer: {
        flex: 8,
        flexDirection: 'row',
        marginVertical: 10,
    },

    container: {
        flex: 1,
        backgroundColor: '#5D6061',
        alignItems: 'center',
        justifyContent: 'center',
    },

    headerContainer: {
        flex: 1,
        flexDirection: 'row',
        marginTop: 10,
        marginHorizontal: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        borderRadius: 10,
        padding: 10,
        justifyContent: 'center',
        alignItems: 'center'
    },

    textLarge: {
        fontfamily: 'Comic Sans MS',
        fontSize: 30,
        color: '#fff'
    },

    textMedium: {
        fontfamily: 'Comic Sans MS',
        flex: 4,
        fontSize: 25,
        color: '#fff',
        textAlign: 'center',
        verticalAlign: 'center',
    },

    textSmall: {
        flex: 4,
        fontfamily: 'Comic Sans MS',
        fontSize: 22,
        color: '#fff',
        textAlign: 'center'
    },

    textTiny: {
        fontfamily: 'Comic Sans MS',
        fontSize: 12,
        color: '#fff'
    },

    columnText: {
        color: '#fff',
        textAlign: 'center'
    },

    columnHyperlink: {
        color: 'cornflowerblue',
        textAlign: 'center'
    },

    columnHeader: {
        flex: 20,
        marginHorizontal: 15,
        justifyContent: 'center',
        alignItems: 'center'
    },

    modalViewContainer: {
        borderRadius: 25,
        backgroundColor: '#5D6061',
    },

    modalButton: {
        flex: 1,
        backgroundColor: '#767676',
        borderRadius: 100,
        marginHorizontal: 15,
        justifyContent: 'center',
        alignItems: 'center',
        height: 100,
        elevation: 5,
    },

    ipInputBox: {
        height: 60,
        fontSize: 50,
        backgroundColor: '#2e3030',
        borderColor: 'gray',
        borderWidth: 1,
        color: '#fff',
        textAlign: 'center',
        verticalAlign: 'center',
        fontfamily: 'Comic Sans MS',
    },

    ipInputBoxMedium: {
        backgroundColor: '#2e3030',
        borderColor: 'gray',
        color: '#fff',
        fontSize: 15,
        textAlign: 'center',
        fontfamily: 'Comic Sans MS',

    },

    rightSideRow: {
        flexDirection: 'row',
        position: 'absolute',
        right: 0
    },

    ArmContainer: {
        flex: 1,
        backgroundColor: '#5D6061'
    },

    buttonBackground: {
        flex: 1,
        maxHeight: 100,
        width: 80,
        minHeight: 50,
        backgroundColor: '#3a3d3d',
        borderRadius: 20,
        elevation: 5,
        justifyContent: 'center',
        alignItems: 'center',
        margin: 20,
        padding: 10
    },

    PaverArmBaseButton: {
        flex: 3,
        marginLeft: 15,
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        marginBottom: 10,
        marginRight: 15,
    },

    PaverArmBackground: {
        borderRadius: 10,
        elevation: 3,
        backgroundColor: '#2e3030',
        marginBottom: 10,
        marginLeft: 10,
        marginRight: 10,
        width: 180,
    },

    mainButtonTextVertical: {
        fontfamily: 'Comic Sans MS',
        fontSize: 15,
        color: '#fff',
        textAlign: 'center',
        bottom: 5,
        paddingTop: 5
    },

    mainButtonTextHorizontal: {
        fontfamily: 'Comic Sans MS',
        fontSize: 15,
        color: '#fff',
        textAlign: 'center',
        bottom: 5,
        paddingVertical: 5
    },

    buttonTextCenter: {
        fontfamily: 'Comic Sans MS',
        fontSize: 22,
        color: '#fff',
        textAlign: 'center',
        paddingTop: 7
    },

    buttonImage: {
        fontfamily: 'Comic Sans MS',
        color: '#fff',
        textAlign: 'center',
    },

    textSmallCenter: {
        textAlign: 'center',
        fontfamily: 'Comic Sans MS',
        fontSize: 18,
        color: '#fff'
    },

    bugText: {
        textAlign: 'center',
        fontfamily: 'Comic Sans MS',
        fontSize: 22,
        color: '#0E86D4'
    },

    devText: {
        fontfamily: 'Comic Sans MS',
        fontSize: 18,
        color: '#fff',
        textAlign: 'center',
    },

    icons: {
        alignItems: 'center',
        alignSelf: 'center',
        justifyContent: 'center',
    },

    evenlySpaceIcons: {
        flexDirection: 'row',
        gap: 30,
    },

});

export default ControllerStyle;

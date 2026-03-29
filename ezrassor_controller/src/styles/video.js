import { NativeStackView } from '@react-navigation/native-stack';
import { StyleSheet } from 'react-native';
import { normalize } from '../functionality/display';
import { Platform } from 'react-native';

const ControllerStyle = StyleSheet.create({

        keyboardAwareScrollView: {
                flexGrow: 1,
                ...Platform.select({
                        ios: {
                                flex: 1
                        }
                })
        },

        buttonContainer: {
                backgroundColor: '#FFFFFF',
                fontfamily: 'Comic Sans MS',
                borderRadius: 8,
                height: '10%',
                width: '12%',
                textAlign: 'center',
                alignSelf: 'flex-end',
                // position: 'absolute',
                left: 10,
                bottom: 10,
                ...Platform.select({
                        ios: { // as same as height
                                // marginRight: 20
                        },
                        android: {
                        }
                })
        },

        buttonModalContainer: {
                backgroundColor: '#FFFFFF',
                fontfamily: 'Comic Sans MS',
                borderRadius: 8,
                height: '10%',
                width: '12%',
                textAlign: 'center',
                alignSelf: 'flex-end',
                position: "absolute",
                right: 10,
                bottom: 20,
                // position: 'absolute',
                // left: 10,
                ...Platform.select({
                        ios: { // as same as height
                                // marginRight: 20
                        },
                        android: {
                        }
                })
        },

        buttonText: {
                fontfamily: 'Comic Sans MS',
                textAlign: 'center',
                fontSize: 50,
                ...Platform.select({
                        ios: {
                                // marginTop: 5, // as same as height
                        },
                        android: {}
                })
        },

        connectButton: {
                flexDirection: 'row',
                backgroundColor: '#FFFFFF',
                display: 'flex',
                overflow: 'hidden',
                justifyContent: 'space-evenly',
                fontfamily: 'Comic Sans MS',
                borderRadius: 8,
                textAlign: 'center',
                verticalAlign: 'center',
                // height: '20%',
                height: (2 * normalize(45, 1.5)),
                lineHeight: (2 * normalize(45, 1.8)),
                width: '30%',
                marginTop: 5,
                marginBottom: 15
        },

        drumFunctionContainer: {
                flex: 6,
                justifyContent: 'center',
                marginHorizontal: 10,
                padding: 10,
                borderRadius: 10,
                elevation: 3,
                backgroundColor: 'transparent',
        },

        headerContainer: {
                flex: 1,
                flexDirection: 'row',
                marginTop: "0.5%",
                marginHorizontal: 0,
                elevation: 3,
                backgroundColor: '#2e3030',
                opacity: 0.8,
                borderRadius: 10,
                paddingHorizontal: 5,
                justifyContent: 'space-around',
                alignItems: 'center',
                marginHorizontal: "1%"
        },

        upAndDownDPad: {
                flex: .7,
                marginHorizontal: 5,
                backgroundColor: '#3a3d3d',
                opacity: 0.8,
                borderRadius: 10,
                margin: 10,
                elevation: 5,
                justifyContent: 'center',
                alignItems: 'center',
        },
        
});

export default ControllerStyle;

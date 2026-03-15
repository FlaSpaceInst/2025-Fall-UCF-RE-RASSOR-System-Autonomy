import React from 'react';
import {
  Text,
  View,
  Image,
  StatusBar,
  Pressable
} from 'react-native';
import ControllerStyle from '../../../src/styles/controller';
import * as Font from 'expo-font';
import logo from '../../../assets/fsiLogo.png';
import * as SplashScreen from 'expo-splash-screen';
import { normalize } from '../../functionality/display';

const CONNECTION_TEXT = `Troubleshooting connection errors:

1. Make sure both the phone and the rover are connected to the same WIFI.

2. Double check you are typing in the right IP address and port when trying to connect.

3. If steps 1 and 2 did not work, try restarting both the rover and the phone
(or restarting the application).
`;

/**
 * React component used as a help screen for users at the <IPConnect> screen.
 */
export default class ConnectionHelp extends React.Component {

  constructor(props) {
    SplashScreen.hideAsync();
    super(props);

    this.state = {
      isLoading: true
    };

    this.connectionText = CONNECTION_TEXT;
  }

  async componentDidMount() {

    this.setState({ isLoading: false });
  }

  redirect() { }

  render() {
    // I.e., don't do full render if font is still loading...
    if (this.state.isLoading) {
      return <View style={{ backgroundColor: '#5D6061' }} />;
    }

    return (
      <View style={ControllerStyle.screenLayout} >

        <StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

        {/* Title container. */}
        <View style={[ControllerStyle.title]}>
          <Text adjustsFontSizeToFit={false} numberOfLines={1} style={[ControllerStyle.titleText, {fontSize: '3.5vw', justifyContent: 'center'}]}>
            RE-RASSOR Connection Help
          </Text>
        </View>

        {/* Body container. */}
        <View style={{ flexDirection: 'row', justifyContent: 'space-between', marginTop: '2%' }}>

          {/* FSI logo. */}
          <Image source={logo} resizeMode= "contain" style={[ControllerStyle.fsiLogo, {height: '45%', alignSelf: 'center', justifyContent: 'center', marginTop: '3%'}]} />

          {/* Inner-body container. */}
          <View adjustsFontSizeToFit={true} backgroundColor="#4a4d4e" width="70%" flexWrap="nowrap" style={ControllerStyle.containerTwo}>
            <Text
              adjustsFontSizeToFit={true}
              numberOfLines={10}
              style={{
                alignSelf: 'center',
                fontfamily: 'Comic Sans MS',
                margin: 5,
                fontSize: '1.3vw',
                color: '#fff'
                }}
            >
              {this.connectionText}
            </Text>
          </View>

        </View>

		{/* Back button. */}
		<Pressable
            style={[ControllerStyle.buttonContainer, {alignSelf:'center', marginTop: '3%'}]}
            onPress={() => {
              this.props.navigation.goBack(null)
            }}
          >
            <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.buttonText, {fontSize: '2vw', fontWeight: '600'}]}>
              Back
            </Text>
          </Pressable>

      </View>
    );
  }
}

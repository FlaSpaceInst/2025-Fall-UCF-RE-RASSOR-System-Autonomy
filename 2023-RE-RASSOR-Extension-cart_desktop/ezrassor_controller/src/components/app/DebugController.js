import React from "react";
import {
  Linking,
  Text,
  View,
  Pressable,
  TouchableHighlight,
} from "react-native";
import Modal from "react-native-modal";
import * as ScreenOrientation from 'expo-screen-orientation';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import PaverArmControllerStyle from '../../../src/styles/controllerPaverArm';
import ControllerStyle from "../../../src/styles/controller";

/**
 * Modal-like React component to show the about-the-app info while in <ControllerScreen>.
 */
export default class InformationController extends React.Component {

  constructor(props) {
    super(props);

    this.state = {
      devModalVisible: false,
      helpModalVisible: false,
    };
  }

  setDevModalVisible(visible) {
    this.setState({ devModalVisible: visible });
  }

  setHelpModalVisible(visible) {
    this.setState({ helpModalVisible: visible})
  }

  componentDidMount() {
  
  }

  render() {
    return (
      <View style={{ flexDirection: "row"}}>

        {/* Left-side of modal. */}
        <View style={ControllerStyle.columnHeader}>

          <Text style = {ControllerStyle.textHeader}>Forwarder status: update later</Text>
          <Pressable
            style={[ControllerStyle.columnText, {color: 'white', marginVertical: 10, backgroundColor: '#3a3d3d',
              borderRadius: 100,
              paddingVertical: 10,
              paddingHorizontal: 20,
              shadowColor:'black',
              shadowOffset: {width: 0, height: 0},
              shadowRadius: 10,
              shadowOpacity: .4,
              selectable: false,
              marginVertical: 40,
              userSelect: 'none'
            }]}
            >
              <Text
                selectable = {false}
                style =  {[{color: 'white'}]}
              >
                Restart Forwarder
              </Text>
          </Pressable>

          <Text style = {ControllerStyle.textHeader}>Processes status: update later</Text>
          <Pressable
            style={[ControllerStyle.columnText, {color: 'white', marginVertical: 10, backgroundColor: '#3a3d3d',
              borderRadius: 100,
              paddingVertical: 10,
              paddingHorizontal: 20,
              shadowColor:'black',
              shadowOffset: {width: 0, height: 0},
              shadowRadius: 10,
              shadowOpacity: .4,
              selectable: false,
              marginVertical: 40,
              userSelect: 'none'
            }]}
            >
            <Text
                selectable = {false}
                style =  {[{color: 'white'}]}
              >
                Restart Processes
              </Text>
          </Pressable>

          <View style={{ marginVertical: 10 }} />


          {/* About-the-team modal. */}
          <Modal
            style={ControllerStyle.modalViewContainer}
            isVisible={this.state.devModalVisible}
            supportedOrientations={['landscape']}
            onSwipeComplete={() => this.setDevModalVisible(false)}
            swipeDirection={["down", "up", "left", "right"]}
            onRequestClose={() => this.setDevModalVisible(false)}
			
          >
            <View style={{ flexDirection: "row" }}>
              <View style={ControllerStyle.columnHeader}>
                <View style={{ marginVertical: 10 }} />
                <Text style={ControllerStyle.textSmall}>EZ-RASSOR</Text>
                <View style={{ marginVertical: 10 }} />
                <View style={{ flexDirection: "row" }}>
                  <View>
                  </View>
                  <View style={{ marginHorizontal: 5 }} />
                </View>
              </View>
              <View
                style={{
                  flex: 0.5,
                  borderRadius: 20,
                  backgroundColor: "#2e3030",
                }}
              ></View>
              <View style={ControllerStyle.columnHeader}>
                <Text style={ControllerStyle.textSmall}>RE-RASSOR CART</Text>
                <View style={{ marginVertical: 10 }} />
                <View style={{ flexDirection: "row" }}>
                  <View>
                    
                  </View>
                </View>
              </View>
            </View>

            {/* Modal X button. */}
            <Pressable
              style={[ControllerStyle.buttonModalContainer]}
              onPress={() => {
                this.setDevModalVisible(false);
              }}
            >
				<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="36" height="36">
					<line x1="18" y1="6" x2="6" y2="18" stroke="black" stroke-width="2" stroke-linecap="round"/>
					<line x1="6" y1="6" x2="18" y2="18" stroke="black" stroke-width="2" stroke-linecap="round"/>
				</svg>
            </Pressable>
          </Modal>

        </View>

        {/* Vertical separating bar. */}
        <View style={{ flex: 0.5, borderRadius: 20, backgroundColor: "#2e3030", }}></View>

        {/* Right-side of modal. */}
        <View style={ControllerStyle.columnHeader}>

        </View>
      </View>
    );
  }
}

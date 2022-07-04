import React from "react";

import styled, { createGlobalStyle } from "styled-components";
import Module from "./views/Module";

const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
        height: 100%;
    }
`

const Layout = styled.main`
    height: 100%;
    display: grid;
    box-sizing: border-box;
    grid-template-columns: 3fr 2fr 5fr 2fr 3fr;
    grid-template-rows: 2fr 1fr;
    grid-template-areas: 
        "left-video view view view right-video"
        "neck neck network car car";
    gap: 16px 16px;
    background-color: rgb(220, 220, 220);
    padding: 8px;
`

const LeftVideo = styled(Module)`
     grid-area: left-video;
`
const View = styled(Module)`
    grid-area: view;
`
const RightVideo = styled(Module)`
    grid-area: right-video;
`
const Neck = styled(Module)`
    grid-area: neck;
`
const Network = styled(Module)`
    grid-area: network;
`
const Car = styled(Module)`
    grid-area: car;
`

export class RoboticAvatar extends React.Component {
    private right_video: React.RefObject<HTMLElement> = React.createRef();
    private right_video_status: React.RefObject<HTMLElement> = React.createRef();

    render(): React.ReactNode {
        return <Layout>
            <GlobalStyle/>
            <LeftVideo title="Left Video Setting" status="error" message="Not Implemented">Not Implemented</LeftVideo>
            <View title="Preview Viewport" status="error" message="Not Implemented">Not Implemented</View>
            <RightVideo title="Right Video Setting" status="error" message="Not Implemented">Not Implemented</RightVideo>

            <Neck title="Neck Robot Setting" status="error" message="Not Implemented">Not Implemented</Neck>
            <Network title="Networking Settings" status="error" message="Not Implemented">Not Implemented</Network>
            <Car title="Car Settings" status="error" message="Not Implemented">Not Implemented</Car>
        </Layout>
    }
}
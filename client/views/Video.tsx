import { Autocomplete, Button, Input, Label, NativeSelect, Slider, Switch, Typography } from "@equinor/eds-core-react";
import React from "react";
import Model from "../models/video"
import Module from "./Module";

interface VideoProps {
    model: Model,
    title: string,
}

export default class Video extends React.Component<VideoProps> {
    render(): React.ReactNode {
        return <Module title={this.props.title} status={"error"} message={"No Video Source Configured"}>
            <Typography variant="h5">Video Source</Typography>
            <Switch label="Show connected sources" disabled />
            <Switch label="Show remote sources" disabled />
            <Autocomplete label="Devices:" options={[]} disabled />
            <Label label="Resolution Height:" />
            <Input type="number" disabled/>
            <Label label="Resolution Width:" />
            <Input type="number" disabled/>
            <Label label="Frame rate:" />
            <Input type="number" disabled/>
            <Button disabled>Apply Resolution Changes</Button>
            <Typography variant="h5">View Settings</Typography>
            <Autocomplete label="Rotate Video" options={["0°", "90°", "180°", "270°"]} initialSelectedOptions={["0°"]} disabled/>
            <Label label="Zoom: (%)" />
            <Slider value={100} min={50} max={200} aria-label="Zoom in %" disabled/>
        </Module>
    }
}
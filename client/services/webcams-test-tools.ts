import { IWebcams } from "./webcams";

/* istanbul ignore next */
export class WebcamsEmpty implements IWebcams{
    getVideoSources(): Promise<MediaDeviceInfo[]> {
        throw new Error("Method not implemented.");
    }
}

/* istanbul ignore next */
export class WebcamsDevice implements IWebcams{
    public devices: MediaDeviceInfo[];

    constructor(...devices: MediaDeviceInfo[]){
        this.devices = devices;
    }

    async getVideoSources(): Promise<MediaDeviceInfo[]> {
        return this.devices;
    }
}

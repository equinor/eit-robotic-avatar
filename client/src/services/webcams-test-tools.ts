import { IWebcams } from "./webcams";

/* istanbul ignore next */
export class WebcamsEmpty implements IWebcams{
    emptyStream: MediaStream = {} as any;
    getStream(deviceId: string): Promise<MediaStream> {
        throw new Error("Method not implemented.");
    }

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
    emptyStream: MediaStream = {} as any;

    getStream(deviceId: string): Promise<MediaStream> {
        throw new Error("Method not implemented.");
    }

    async getVideoSources(): Promise<MediaDeviceInfo[]> {
        return this.devices;
    }
}

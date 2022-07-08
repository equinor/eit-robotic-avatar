export interface IWebcams {
    emptyStream: MediaStream;
    getStream(deviceId: string): Promise<MediaStream>;
    getVideoSources(): Promise<MediaDeviceInfo[]>
}

/* istanbul ignore next */
export class Webcams implements IWebcams{
    emptyStream: MediaStream = new MediaStream();
    async getStream(deviceId: string): Promise<MediaStream> {
        return navigator.mediaDevices.getUserMedia({video: {deviceId: deviceId} , audio: false});
    }

    async getVideoSources(): Promise<MediaDeviceInfo[]> {
        // Experimental Chrome only api
        let status = await navigator.permissions.query({name:'camera'} as any);
        if (status.state == "granted") {
            return await navigator.mediaDevices.enumerateDevices()
        }
        if (status.state == "prompt") {
            await navigator.mediaDevices.getUserMedia({video: true, audio: false});
            return await this.getVideoSources();
        }
        console.warn("User have denied the use of connected webcams!");
        return [];
    }
}
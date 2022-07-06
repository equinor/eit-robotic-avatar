import {IWebcams} from "../services/webcams";

export interface VideoSource {
    id: string,
    type: "remote" | "local"
    label: string
}

export default class Video{
    private _show_local = false;
    private _stream: MediaStream = new MediaStream();
    public onStream: () => void = noop;

    constructor(private _webcams: IWebcams){

    }

    get show_local(): boolean {
        return this._show_local;
    }

    set show_local(show: boolean) {
        if (this._show_local !== show){
            this._show_local = show;
        }
    }

    get stream(): MediaStream {
        return this._stream;
    }

    async getSources(): Promise<VideoSource[]> {
        return this.getLocalSources();
    }

    async setSource(source:VideoSource) {
        let stream: MediaStream;
        if (source.type === "local") {
            stream = await this.getLocalStream(source.id);
        } else {
            throw new Error("Remote source not implemented");
        }

        this._stream = stream;
        this.onStream();
    }

    private async getLocalSources(): Promise<VideoSource[]>{
        if(this._show_local){
            let devices = await this._webcams.getVideoSources();
            return devices
                .filter((device) => device.kind == "videoinput")
                .map((device) => {
                    let label = `${device.label} (${device.deviceId.substring(0,6)})`
                    return {   
                        id: device.deviceId,
                        type: "local",
                        label: label 
                    }
                })
        }else {
            return [];
        }
    }

    private async getLocalStream(deviceId: string): Promise<MediaStream> {
        return this._webcams.getStream(deviceId);
    }
}

const noop = () => {};
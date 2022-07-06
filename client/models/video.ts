import {IWebcams} from "../services/webcams";

export interface VideoSource {
    id: string,
    type: "remote" | "local"
    label: string
}

export default class Video{
    private _show_local = false;

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

    async getSources(): Promise<VideoSource[]> {
        return this.getLocalSources();
    }

    async getLocalSources(): Promise<VideoSource[]>{
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
}
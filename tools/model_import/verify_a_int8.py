#!/usr/bin/env python3
# Compare real C++ board output with supplied Python decoder and SDK fixtures.
import argparse
import importlib.util
import json
from pathlib import Path
import cv2
import numpy as np

def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--package",type=Path,required=True)
    parser.add_argument("--results",type=Path,required=True)
    args=parser.parse_args()
    decode=load_module("handoff_decode",args.package/"host_decode.py")
    config=json.loads((args.package/"runtime_config.json").read_text())
    manifest=json.loads((args.package/"fixtures/manifest.json").read_text())
    cpp=json.loads((args.results/"image_results.json").read_text())
    records=[]
    for expected,actual in zip(manifest,cpp,strict=True):
        assert expected["image"]==actual["image"]
        stem=Path(expected["image"]).stem
        frame=cv2.imread(str(args.package/expected["image"]))
        rgb,gain,pad=decode.letterbox(frame)
        board_rgb=np.fromfile(args.results/(stem+".rgb"),np.uint8).reshape(rgb.shape)
        assert np.array_equal(rgb,board_rgb),"BGR/RGB or letterbox mismatch"
        values=np.fromfile(args.results/(stem+".f32"),np.float32).reshape(1,26,8400)
        outputs=[values[:,:4],values[:,4:6],values[:,6:22],values[:,22:26]]
        py,route=decode.decode(outputs,gain,pad,frame.shape,config["router"])
        by_class={d["cls"]:d for d in actual["raw"]}
        reference={d["cls"]:d for d in expected["SDK_expected_raw"]}
        cpp_error=0.;sdk_xy=[];sdk_score=[];sdk_visibility=[];sdk_box=[]
        for d in py:
            c=d["cls"];target=by_class[c]
            for field in ("score","box","kpt"):
                error=float(np.max(np.abs(np.asarray(d[field])-np.asarray(target[field]))))
                cpp_error=max(cpp_error,error)
            if c in reference:
                a=np.asarray(target["kpt"]);b=np.asarray(reference[c]["kpt"])
                sdk_xy.extend(np.linalg.norm(a[:,:2]-b[:,:2],axis=1).tolist())
                sdk_score.append(abs(target["score"]-reference[c]["score"]))
                sdk_visibility.extend(np.abs(a[:,2]-b[:,2]).tolist())
                sdk_box.extend(np.abs(np.asarray(target["box"])-reference[c]["box"]).tolist())
        assert len(py)==len(actual["raw"]),"fixture unexpectedly has multiple same-class detections"
        assert cpp_error<1e-3,(stem,cpp_error)
        assert route["counts"]==actual["route"]["counts"]
        assert abs(route["weight_far"]-actual["route"]["weight_far"])<1e-6
        record={"image":expected["image"],"cpp_python_max_abs_error":cpp_error,
            "preprocess_pixels_identical":True,"board_counts":route["counts"],
            "sdk_counts":expected["SDK_expected_route"]["counts"],
            "board_weight_far":route["weight_far"],"sdk_weight_far":expected["SDK_expected_route"]["weight_far"],
            "sdk_keypoint_mean_original_px":float(np.mean(sdk_xy)) if sdk_xy else None,
            "sdk_keypoint_max_original_px":max(sdk_xy,default=None),
            "sdk_score_max_abs_error":max(sdk_score,default=None),
            "sdk_visibility_max_abs_error":max(sdk_visibility,default=None),
            "sdk_box_max_abs_error_original_px":max(sdk_box,default=None),
            "reference_is_human_GT":False}
        records.append(record)
        print(stem,"cpp_python_error",cpp_error,"SDK_max_point_delta_original_px",record["sdk_keypoint_max_original_px"])
    (args.results/"image_comparison.json").write_text(json.dumps(records,indent=2)+"\n")
    times={key:[row[key] for row in cpp] for key in ("preprocess_ms","inference_ms","decode_ms","main_pipeline_ms")}
    (args.results/"performance.json").write_text(json.dumps({"sample_count":len(cpp),"warmup_excluded":False,
        "method":"steady_clock, six independent images; no control latency measurement",
        "timing_ms":{key:{"mean":float(np.mean(v)),"max":max(v)} for key,v in times.items()}},indent=2)+"\n")
if __name__=="__main__":main()

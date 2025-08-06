#!/usr/bin/env python3
import json, subprocess, time
from typing import List, Dict, Any
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Run:  uvicorn gemma_proxy:app --host 0.0.0.0 --port 8000
# Requires: `ollama run gemma3:4b` working on the host.

MODEL = "gemma3:4b"
JSON_ONLY = "Return ONLY a JSON list (e.g., [3,1,2]). No prose."

class Candidate(BaseModel):
    index: int
    pred_ms: float
    steps: Any | None = None  # plan as nested list (optional)

class RerankIn(BaseModel):
    ctx: Dict[str, Any]  # {size, clutter, heat, gas, noise}
    candidates: List[Candidate]

app = FastAPI()
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

#    “/scored.json”, “/good_rewrites.json”, etc. all work:
app.mount(
    "/",
    StaticFiles(directory="..", html=True),
    name="static",
)

@app.post("/api/rerank")
async def rerank(req: RerankIn):
    lines = [
        f"CTX: {req.ctx.get('size'):.1f},{req.ctx.get('clutter'):.2f},{int(bool(req.ctx.get('heat')))},{int(bool(req.ctx.get('gas')))}, {int(bool(req.ctx.get('noise')))}",
        "GOAL: pick best tradeoff of speed vs hazard. Prefer lower pred_ms unless heat/gas imply sensing first. Avoid redundant waits.",
        JSON_ONLY, "CANDIDATES:"
    ]
    for c in req.candidates:
        # Compact signature of up to 8 tokens
        sig = ""
        if isinstance(c.steps, list):
          flat = []
          for step in c.steps:
            if isinstance(step, list):
              flat += step
          flat = flat[:8]
          abbr = {'lidar_scan':'L','thermal_snap':'T','gas_sniff':'G','audio_probe':'A','wait':'W'}
          sig = "".join([abbr.get(s, s[:1].upper()) for s in flat])
        lines.append(f"{c.index}:{int(c.pred_ms)},{sig}")
    prompt = "\n".join(lines)

    try:
        t0=time.time()
        proc = subprocess.run(["ollama","run",MODEL], input=prompt, text=True, capture_output=True, timeout=15)
        txt = proc.stdout.strip()
        if txt.startswith("```"):
            txt = txt.strip("`").strip()
            if "\n" in txt: txt = txt.split("\n",1)[1].strip()
        out = json.loads(txt)
        if not (isinstance(out, list) and all(isinstance(i,int) for i in out)):
            raise ValueError("LLM response not a list of ints")
        return out
    except Exception as e:
        return []  # graceful fallback

"""Pi 에서 정책 forward 1회 시간을 잰다 — ACT · SmolVLA 공용.

데스크탑이 만든 run_on_pi.py(ACT 전용)를 정책 종류에 무관하게 돌도록 넓힌 것이다.

## 판정 기준 — 정책마다 자기 청크 수명이 잣대다

`select_action` 은 `n_action_steps` 마다 **한 번만** forward 를 돈다. 그 사이는
큐에서 꺼내 쓴다. 그래서 forward 1회 시간이 **청크 수명보다 짧아야** 정책이
자기 청크를 따라간다.

    ACT      n_action_steps=100  →  30fps 에서 3.33초
    SmolVLA  n_action_steps= 50  →  30fps 에서 1.67초   ← 예산이 절반이다

**둘을 같은 3.3초 잣대로 재면 SmolVLA 를 후하게 평가하게 된다.** 여기서는 각
체크포인트의 `n_action_steps` 를 읽어 자기 기준으로 판정한다.

## device 는 반드시 덮어써야 한다

학습 때 device(xpu)가 **전처리기 파이프라인에 박혀** 저장된다. 정책 config 에는
폴백이 있지만 전처리기에는 없다. 안 덮으면 이렇게 죽는다.

    RuntimeError: Expected all tensors to be on the same device,
                  but got mat1 is on xpu:0, different from other tensors on cpu

    python3 bench_policy_pi.py <체크포인트경로>
    python3 bench_policy_pi.py <경로> --res 240 320     # 해상도를 바꿔 재기
"""
import argparse
import json
import os
import statistics
import time


def load_policy(ckpt, device):
    """config.json 의 type 을 보고 맞는 정책을 만든다.

    ⚠️ `ACTConfig.from_pretrained` 를 직접 부르면 안 된다. 체크포인트의
    config.json 에 `type` 필드가 들어 있는데 구체 클래스는 그걸 모르는 필드로 보고
    거부한다(lerobot 0.4.4 · draccus DecodingError). `type` 을 해석해 알맞은
    클래스로 넘기는 건 **기반 클래스** 쪽이다.

        DecodingError: The fields `type` are not valid for ACTConfig

    그리고 `import lerobot.policies` 를 먼저 해야 선택지 등록이 끝난다 —
    안 하면 "Couldn't find a choice class for 'act'" 로 죽는다.
    """
    import lerobot.policies  # noqa: F401  (선택지 등록)
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class

    cfg = PreTrainedConfig.from_pretrained(ckpt)
    cfg.device = device
    policy = get_policy_class(cfg.type).from_pretrained(ckpt, config=cfg)
    return cfg.type, cfg, policy.to(device).eval()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--res", nargs=2, type=int, default=None, metavar=("H", "W"))
    ap.add_argument("--runs", type=int, default=5)
    args = ap.parse_args()

    import torch
    from lerobot.policies.factory import make_pre_post_processors

    torch.set_num_threads(os.cpu_count() or 4)
    kind, cfg, policy = load_policy(args.ckpt, args.device)

    # 학습 때 device 가 전처리기에 박혀 있다 — 위 docstring 참고.
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=args.ckpt,
        preprocessor_overrides={"device_processor": {"device": args.device}},
    )

    img_keys = [k for k in cfg.input_features if "image" in k]
    c, h, w = cfg.input_features[img_keys[0]].shape
    if args.res:
        h, w = args.res
    state_dim = cfg.input_features["observation.state"].shape[0]
    chunk_s = cfg.n_action_steps / 30.0

    print(f"체크포인트 {args.ckpt}")
    print(f"  정책 {kind}  파라미터 {sum(p.numel() for p in policy.parameters())/1e6:.1f}M")
    print(f"  카메라 {len(img_keys)}대  입력 ({c}, {h}, {w})")
    print(f"  chunk_size={cfg.chunk_size}  n_action_steps={cfg.n_action_steps}"
          f"  → 청크 수명 {chunk_s:.2f}초")
    extra = getattr(cfg, "resize_imgs_with_padding", None)
    if extra:
        print(f"  ⚠️ 내부 리사이즈 {extra} — 입력 해상도를 낮춰도 여기까지만 줄어든다")
    print(f"  torch {torch.__version__}  threads {torch.get_num_threads()}\n")

    def make_obs():
        obs = {"observation.state": torch.zeros(1, state_dim), "task": ["pick up the queen"]}
        for k in img_keys:
            obs[k] = torch.rand(1, c, h, w)
        return obs

    times = []
    for r in range(args.runs + 1):
        policy.reset()
        obs = make_obs()
        t = time.perf_counter()
        with torch.no_grad():
            post(policy.select_action(pre(obs)))
        dt = time.perf_counter() - t
        if r == 0:
            print(f"워밍업 (버림): {dt*1000:.0f} ms")
        else:
            times.append(dt)

    med = statistics.median(times)
    ratio = med / chunk_s
    print(f"\n=== forward 1회 ({h}x{w}, {args.runs}회) ===")
    print("  " + "  ".join(f"{x*1000:.0f}ms" for x in times))
    print(f"  중앙값       {med*1000:8.0f} ms")
    print(f"  스텝당 상각  {med*1000/cfg.n_action_steps:8.2f} ms")
    print(f"  자기 청크({chunk_s:.2f}초) 대비 {ratio*100:6.0f} %"
          f"   {'🔴 청크를 못 따라감' if ratio >= 1.0 else ('🟠 여유 부족' if ratio > 0.5 else '✅')}")


if __name__ == "__main__":
    main()

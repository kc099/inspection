import onnxruntime as ort
import sys

path = sys.argv[1]
sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
print("INPUTS:")
for i, inp in enumerate(sess.get_inputs()):
    print(f"  [{i}] {inp.name}: shape={inp.shape} type={inp.type}")
print("OUTPUTS:")
for i, out in enumerate(sess.get_outputs()):
    print(f"  [{i}] {out.name}: shape={out.shape} type={out.type}")

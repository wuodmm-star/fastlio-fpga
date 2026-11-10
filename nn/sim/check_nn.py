exp = [l.split() for l in open("nn_expect.txt")]
out = [l.split() for l in open("nn_out.txt")]
bad = sum(a != b for a, b in zip(exp, out))
print(f"expected {len(exp)} search results, got {len(out)}, mismatching {bad}")
for k, (a, b) in enumerate(zip(exp, out)):
    if a != b:
        print("first mismatch at search", k, "\n exp", " ".join(a), "\n got", " ".join(b)); break
print("ALL BIT-EXACT" if bad == 0 and len(exp) == len(out) else "FAIL")

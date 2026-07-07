# NMC-CXL accelerator — thin wrapper over scripts/run_all.sh
.PHONY: sim lint wave test results clean

sim:
	bash scripts/run_all.sh

lint:
	bash scripts/run_all.sh --lint-only

wave:
	bash scripts/run_all.sh --wave

# single test: make test T=test_sparse_attention
test:
	bash scripts/run_all.sh --test $(T)

# perf tests + refresh docs/results.json (rendered by docs/index.html)
results:
	bash scripts/run_all.sh --json docs/results.json

clean:
	rm -rf obj_dir build/*.log build/waves.vcd

# Backend change validation plan

更新日: 2026-08-16

> **現行計画。** `picoem-picocalc` の現行mainにあるDMA／audio観測変更と
> feature-gated OPT4候補を、既存のPicoCalc exactness契約へ安全に接続するための
> 実行順序を定義する。完了するまでbackend pinやpromoted targetを更新しない。

## 判定

実装は継続可能であり、技術的な行き止まりはない。OPT4-Aのsentinel回帰と、DMA／audioの
quantum-invariance比較は、backend commit `6a675b1`でローカル合格した。ただし、format／
Clippy gate、CLI end-to-end、firmware再回帰は未完了なので、promotion可能な状態ではない。

- `rp2040-emu` unit test 1246件は合格する。
- DMA quantum-invariance integration test 5件は合格する。
- `picocalc-harness --features event-horizon-profiler`のunit test 67件は合格する。
- HIGH_PRIORITY／timer競合の量子幅integration test 5件を追加し、DMA quantum-invarianceは
  合計10/10で合格する。
- 量子幅比較は`Emulator::run`の実行サイクルが命令境界でovershootする契約を考慮し、
  1／16／64の全実行を実際の共通198 master-cycle境界で比較する。
- 最終tick窓だけを表す`timer_due_cycle`／`timer_window_*`は量子幅で窓の分割が変わるため、
  累積値とは分離して内部整合性を検査する。audio sinkのPCM／due-cycle／block／latency
  digestは完全一致を要求する。
- CIと同じ範囲のfmt check、`-D warnings`のClippy、`picocalc_emu`のportable verify、
  新backendでのfirmware exactnessは後続stepで閉じる。

## 絶対条件

1. exactnessを性能、診断の便利さ、実装済み工数より優先する。
2. 旧versioned validationと時点証拠を書き換えない。
3. feature-gated候補をdefault buildやactive targetへ自動的に昇格しない。
4. GitHub Actionsを通常の修正・試行錯誤に使用しない。検証はローカルで行う。
5. `--all-features`を正式な検証コマンドにしない。cache表現には意図的に相互排他な
   featureがあるため、有効なfeature組合せを個別に試験する。
6. backend commitを含むnormalized reportの変化と、firmware挙動の変化を区別する。

## 実行順序

### 1. OPT4-A empty-sentinel回帰を閉じる — 完了 2026-08-16

`DecodedOp`の通常12-byte表現でも、empty entryがfaulting PC `u32::MAX`へ一致しないようにする。
少なくとも次を確認する。

- default representationでempty sentinelを除外する。
- `decoded-op-8byte-prototype`でも同じ意味を維持する。
- faulting PCがcache hit扱いされず、bus faultを発生させる。
- cacheable PCの通常hit、non-cacheable PCのmiss、invalidationを退行させない。
- `unconditional-cache-lookup-prototype`単独の`rp2040-emu`／harness testを合格させる。

source commit `37c50e6`で通常12-byte表現の`matches_pc`にもempty sentinel除外を追加した。
default、`unconditional-cache-lookup-prototype`、`decoded-op-8byte-prototype`、
`compact-dispatch-key-prototype`の各lib testと、unconditional harness testはローカルで合格した。
過去の隔離commitで得た性能・firmware記録は履歴証拠として保持するが、step 2以降の低レベル／
CLI／firmware回帰を閉じるまで、現行mainをpromoted bank候補へ戻さない。

### 2. DMA quantum-invarianceの比較状態を拡張する — 完了 2026-08-16

現在のintegration testが比較するdestination data、channel state、DMA IRQ stateに加え、
今回の通常経路変更が影響し得る次の状態を比較する。

- DMA timer register／fixed-point accumulator
- timer event count、miss countと分類別counter
- selected timer due-cycleとwindow内event状態
- audio sink DMA write count、PCM digest
- block boundary、unexpected gap、service latency
- chain／ringの結果を専用fieldまたは明示assertionで確認する

公開文書に列挙する比較対象は、テストが実際にassertするfieldだけとする。

backend commit `6a675b1`で、公開`DmaSchedulerSnapshot`を追加し、timer register／
accumulator／event・miss分類、audio sinkのDMA write count、PCM digest、block境界、
unexpected gap、service latencyをintegration testへ接続した。timer-paced transferと
fixed-destination PWM audio fixtureを追加し、FORCE、timer、競合、chain／ringを量子
1／16／64で比較して5/5合格した。`Emulator::run`のrequested-cycle overshootをそのまま
比較すると誤検出になるため、テストは共通の実行境界を明示し、window-localな観測値は
「量子幅不変な累積契約」と混同しないよう整合性だけを検査する。

### 3. HIGH_PRIORITYとtimer競合を局所試験する — 完了 2026-08-16

量子`1`、`16`、`64`で少なくとも次を比較する。

- high-priority対normalのFORCE転送
- high-priority timer対normal FORCE
- 複数timer DREQの同cycle競合
- PicoCalc audio timerと別DMA channelの競合
- 同一priority tier内のlowest-channel tie-break
- chain後にready setまたはpriority tierが変わるケース

最終メモリだけでなく、選択channel、timer消費、IRQ、audio観測も一致条件へ含める。

backend commit `00b05f5`で5件を追加した。高優先度FORCE対normal FORCE、高優先度timer対
normal FORCE、同一timer周期のlowest-channel tie-break、PicoCalc audio timer対normal FORCE、
chain後のpriority tier変化を、量子1／16／64で実行した。転送結果・channel残量・timer event／
miss分類・IRQ・audio sinkを比較し、10/10 workloadが合格した。

### 4. report／CLIのend-to-end試験を追加する

次の公開経路をhelper単体ではなく、runnerの引数解析からartifact生成まで通す。

- 非ゼロのtimer miss値がschema-8 reportへ正しいfield名で伝播する。
- `--board none --audio-analysis --audio-wav`でDMA→PWM fixtureを実行する。
- observed sample rate、WAV header、audio sink report、PCM digestを照合する。
- UART markerが複数回のdrainへ分割されてもprofileを開始する。
- marker未観測、scenario終了、cycle limitとの境界をfail-closedで扱う。

board-less audio用fixtureは外部workspaceへ依存させない。source、ライセンス、生成方法を
repository内で固定し、バイナリだけを根拠にしない。

### 5. 文書を実装とテストへ同期する

- event profileの`start_cycle`はfirmwareのUART送信cycleではなく、runnerがdrainしたUART列から
  markerを認識しprofileを有効化したcycleであることを明記する。
- quantum-invariance testの比較fieldを実装と同じ一覧にする（step 2で同期済み）。
- 有効なfeature test matrixと、`--all-features`が不適切な理由を記録する。
- 未使用の`tick_bulk`を参照実装として保持するなら役割を記録し、役割がなければ後続変更で削除する。

### 6. format／Clippy gateを閉じる

機能変更とテスト追加が固まった後に一度だけ整形し、次をローカルで合格させる。

```bash
cargo fmt -p picocalc-board -p picocalc-harness --check
rustfmt --edition 2024 --check \
  crates/rp2040-emu/src/dma.rs \
  crates/rp2040-emu/src/audio_sink.rs
cargo clippy --locked -p picocalc-board -p picocalc-harness \
  --all-targets -- -D warnings
cargo clippy --locked -p rp2040-emu \
  --features event-horizon-profiler --lib -- -D warnings
```

現時点で確認済みのClippy残件は`ssi_flash.rs`の`manual_is_multiple_of`と
`nvic.rs`の`needless_return`である。直近DMA commit由来ではないが、現行HEADのgateとして閉じる。

### 7. 新backendで既存firmwareをローカル再回帰する

旧target recordを変更せず、新backend専用checkoutまたは新しいversioned validation候補で、
少なくとも次を再実行する。

- PicoTetris OPT1-B
- PicoCalc audio
- multicore
- PSRAM
- SD／FAT32
- PicoEdit
- 代表的なofficial Hello／template firmware

backend identityを含む`normalized_report_sha256`は新commitで変わるため、それだけを挙動差と
判定しない。cycle、virtual time、timeline、UART、framebuffer、PSRAM、scenario、
behavior SHA、event-domain digest、audio PCM／due-cycle／block情報を比較する。

### 8. 結果を分類してからpromotionを判断する

| 結果 | 処置 |
|---|---|
| 全ての挙動が一致 | 新しいversioned validation候補を作成できる。pin更新は別判断とする。 |
| 意図したDMA改善だけが変化 | 根拠を記録し、影響するaudio／DMA targetは必要に応じて実機相関する。 |
| 無関係なdomainが変化 | promotionせず原因を修正する。 |
| exactnessが不一致 | 性能や診断上の利点にかかわらず候補を不採用にする。 |

## 正式なローカルgate

最低限、次を全て合格させる。

```bash
cargo test --locked --workspace
cargo test --locked -p rp2040-emu --test dma_quantum_invariance
cargo test --locked -p picocalc-harness --bin picocalc-run \
  --features event-horizon-profiler
```

これに加えて、各実験featureを単独または明示的に許可した組合せで実行する。
相互排他featureをまとめる`cargo test --all-features`は合格条件にしない。
`picocalc_emu`側では`python3 tools/picocalc.py verify`を合格させ、step 7のfirmware
再回帰をportable verifyとは別に完了する。

## 完了条件

- OPT4-Aのsentinel regressionが再現しない。
- DMA／audio／UART markerの新しい公開挙動が直接テストされている。
- source、公開文書、テストの主張が一致している。
- default workspace、正式feature matrix、fmt、Clippyがローカルで全て合格する。
- 新backendによる既存firmwareの挙動差が分類され、未説明の差がない。
- それまでは既存のpromoted backendとtarget pinを維持する。

見積もりは通常2〜3作業日である。deterministic audio fixtureの作成やfirmware回帰で
新しい挙動差が見つかった場合は、追加で1〜2作業日を見込む。

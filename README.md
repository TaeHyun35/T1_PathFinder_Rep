# T1_PathFinder_Rep

Physical AI 1팀 **PathFinder** — 기계 손(로봇 핸드) 프로젝트 저장소

> 세부 주제는 `docs/01_idea/`에서 논의 중입니다. 확정되면 이 문단을 갱신합니다.

## 팀원

| 이름 | GitHub ID | 역할 |
|---|---|---|
| | @TaeHyun35 | 저장소 관리 |
| | | |
| | | |

## 폴더 구조

```
T1_PathFinder_Rep/
├── docs/
│   ├── 01_idea/            # 아이디어·기획
│   ├── 02_research/        # 자료조사·레퍼런스
│   ├── 03_design/          # 설계 (HW·SW 구조)
│   ├── 04_implementation/  # 구현사항·실험 기록
│   ├── 05_management/      # 회의록·결정사항·역할
│   └── 06_progress/        # 전체 계획 + 진행 기록 (⚠ 추가만 가능)
├── src/                    # 코드
├── hardware/               # CAD, 회로도, 부품 목록
└── .github/                # PR 템플릿, 자동 검사
```

| 폴더 | 내용 |
|---|---|
| [01_idea](docs/01_idea/) | 아이디어 제안, 비교, 최종 주제 선정 근거 |
| [02_research](docs/02_research/) | 논문·오픈소스·부품 조사 |
| [03_design](docs/03_design/) | 시스템 구조도, 관절 매핑, 통신·모듈 설계 |
| [04_implementation](docs/04_implementation/) | 기능별 구현 내용, 실험 조건·결과, 문제 해결 |
| [05_management](docs/05_management/) | 회의록, 결정사항, 역할 분담 |
| [06_progress](docs/06_progress/) | 전체 계획(버전별), 개인 진행 기록 |

## ⚠ 06_progress 작성 규칙 (필수)

**`docs/06_progress/`의 기존 파일은 누구도 수정·삭제·이름 변경할 수 없습니다. 본인이 진행한 내용을 새 파일로 추가만 합니다.**

1. 진행 기록은 `docs/06_progress/log/YYYY-MM-DD_본인GitHubID.md` **새 파일**로 올립니다.
   같은 날 두 번째부터는 `_2`, `_3`을 붙입니다. (예: `2026-11-05_TaeHyun35_2.md`)
2. **본인 ID로 된 파일만** 만들 수 있습니다. 다른 사람의 진행 내용을 대신 적지 않습니다.
3. 전체 계획을 바꿀 때는 기존 파일을 고치지 않고 `plan/plan_vN.md`를 새로 만듭니다. 가장 큰 번호가 최신 계획입니다.
4. 이전 기록이 틀렸으면 고치지 말고, 다음 기록에 "정정" 항목으로 적습니다.

이 규칙은 PR마다 [`progress-guard`](.github/workflows/progress-guard.yml) 검사가 자동으로 확인하며, 위반한 PR은 merge할 수 없습니다. 자세한 내용은 [06_progress/README.md](docs/06_progress/README.md)를 보세요.

## 작업 방식

1. 할 일은 **이슈**로 만들고 담당자를 지정합니다.
2. `main`에서 브랜치를 따서 작업합니다: `feat/설명`, `fix/설명`, `docs/설명`
3. 커밋 메시지 접두어: `feat:` `fix:` `docs:` `refactor:` `exp:`(실험) `chore:`
4. **PR**을 올리고 리뷰 1명 + 자동 검사 통과 후 merge합니다. `main`에 직접 push하지 않습니다.
5. 작업한 날은 `06_progress/log/`에 본인 진행 기록을 추가합니다.

## 파일 규칙

- 문서 이름: `분류_내용.md`, 날짜가 중요한 문서는 `YYYY-MM-DD`를 앞에 붙입니다.
- 문서 속 이미지는 같은 폴더의 `assets/`에 넣습니다.
- 데이터셋·모델 가중치·영상 등 큰 파일은 git에 올리지 않고 공유 드라이브 링크로 남깁니다.

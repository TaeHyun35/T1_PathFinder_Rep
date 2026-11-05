#!/usr/bin/env bash
# 06_progress 추가 전용 규칙 검사
# 사용: progress_guard.sh <base-ref> <head-ref> <pr-author>
set -uo pipefail

BASE="$1"; HEAD="$2"; AUTHOR="$3"
DIR="docs/06_progress"
author_lc="$(printf '%s' "$AUTHOR" | tr '[:upper:]' '[:lower:]')"
fail=0
err() { echo "::error file=$2::$1"; echo "  ✗ $2 — $1"; fail=1; }

changes="$(git -c core.quotepath=false diff --name-status -M "$BASE...$HEAD" -- "$DIR")"
if [ -z "$changes" ]; then
  echo "06_progress 변경 없음 — 통과"
  exit 0
fi

echo "06_progress 변경 내역 (PR 작성자: $AUTHOR)"
echo "$changes" | sed 's/^/  /'
echo

while IFS=$'\t' read -r status path1 path2; do
  [ -z "$status" ] && continue
  case "$status" in
    A) ;;
    M)
      if [ "$path1" = "$DIR/README.md" ]; then
        echo "  ! $path1 — 규칙 문서 수정 (CODEOWNERS 리뷰 필요)"
        continue
      fi
      err "기존 파일 수정 금지. 새 파일로 추가하세요" "$path1"; continue ;;
    D) err "기존 파일 삭제 금지" "$path1"; continue ;;
    R*) err "기존 파일 이름 변경·이동 금지 ($path1 → $path2)" "$path1"; continue ;;
    *) err "허용되지 않는 변경($status)" "$path1"; continue ;;
  esac

  rel="${path1#"$DIR"/}"
  name="$(basename "$rel")"
  name_lc="$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]')"
  case "$rel" in
    log/assets/*)
      if ! printf '%s' "$name_lc" | grep -Eq "^[0-9]{4}-[0-9]{2}-[0-9]{2}_${author_lc}_[^/]+\.[a-z0-9]+$"; then
        err "이미지 이름은 YYYY-MM-DD_${AUTHOR}_설명.확장자 형식이어야 합니다" "$path1"
      fi ;;
    log/*/*)
      err "log/ 아래에는 assets/ 외 하위 폴더를 만들 수 없습니다" "$path1" ;;
    log/*)
      if ! printf '%s' "$name_lc" | grep -Eq "^[0-9]{4}-[0-9]{2}-[0-9]{2}_${author_lc}(_[0-9]+)?\.md$"; then
        err "진행 기록은 YYYY-MM-DD_${AUTHOR}.md (같은 날 추가분은 _2, _3) 형식, 본인 ID만 가능합니다" "$path1"
      fi ;;
    plan/*)
      if ! printf '%s' "$rel" | grep -Eq '^plan/plan_v[0-9]+\.md$'; then
        err "계획 파일은 plan/plan_vN.md 형식이어야 합니다" "$path1"
      fi ;;
    README.md) ;;  # 최초 생성
    *)
      err "06_progress에는 plan/, log/ 아래에만 파일을 추가할 수 있습니다" "$path1" ;;
  esac
done <<< "$changes"

echo
if [ "$fail" -ne 0 ]; then
  echo "❌ 06_progress 규칙 위반 — docs/06_progress/README.md를 확인하세요"
  exit 1
fi
echo "✅ 06_progress 규칙 통과"

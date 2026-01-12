# robocup_26_RCKO_src

**robocup_26_RCKO_src** is the main repository for managing RoboCup 2026 (RCKO) competition-related packages and configurations.  
이 레포는 **RoboCup 2026 대회 관련 패키지/설정(특히 파라미터)** 을 관리하는 메인 레포입니다.

---

## 🚀 Usage / 사용 방법

### Clone (with submodules) / 서브모듈 포함 클론 (필수)
This repository uses Git submodules. Please clone with `--recurse-submodules` so that all packages are downloaded.  
이 레포는 **Git submodule**을 사용합니다. 아래 명령으로 클론해야 각 패키지 내용까지 함께 내려옵니다.

```bash
git clone --recurse-submodules <ROOT_REPO_URL>
cd robocup_26_RCKO_src
```

### pull 받을 때

```bash
git pull
git submodule update --init --recursive
```

### If you already cloned / 이미 클론했다면
```bash
git submodule update --init --recursive
```

## 🎯 Purpose / 목적
- Manage competition packages, launch files, configs, and documentation in one place.  
  대회에 필요한 패키지/런치/설정/문서를 한 곳에서 관리합니다.
- Keep robot-specific parameters separated by **NUC-based branches**.  
  로봇(NUC) 별 파라미터 충돌을 막기 위해 **NUC 번호 기준 브랜치**로 분리합니다.

---

## 🌿 Branch Policy / 브랜치 규칙 (중요)

### ✅ Each member MUST create and use a branch that matches their NUC number.
단원들은 **자신의 NUC 번호에 맞는 브랜치**를 만들어 그 브랜치에서 파라미터를 관리하세요.

**Branch naming convention / 브랜치 이름 규칙**
- `nuc1`, `nuc2`, `nuc3` ... 


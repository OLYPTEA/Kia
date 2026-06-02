; Inno Setup script for Kia Studio — packages the PyInstaller onedir build into a setup.exe.
; Prereq: build first  ->  python packaging/build.py   (creates dist/KiaStudio/)
; Compile:  iscc packaging/installer.iss   (install Inno Setup: winget install JRSoftware.InnoSetup)
; Output:   packaging/installer_out/KiaStudio-Setup-<version>.exe

#define AppName "Kia Studio"
#define AppVersion "0.1.0"
#define AppPublisher "Kia Robotics"
#define AppExe "KiaStudio.exe"

[Setup]
AppId={{8F2C9A14-7B3E-4C6A-9E21-KIASTUDIO0001}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
UninstallDisplayIcon={app}\{#AppExe}
SetupIconFile=kia.ico
OutputDir=installer_out
OutputBaseFilename=KiaStudio-Setup-{#AppVersion}
Compression=lzma2/max
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
WizardStyle=modern
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog commandline

[Languages]
Name: "fr"; MessagesFile: "compiler:Languages\French.isl"
Name: "en"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "..\dist\KiaStudio\*"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
